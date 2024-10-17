import serial
import time
import joblib
import logging
import threading
import tkinter as tk
from tkinter import scrolledtext, Canvas, ttk
import queue
import random
import asyncio
import os
from concurrent.futures import ThreadPoolExecutor
import openai
import sys

try:
    import yaml
except ImportError:
    print("The 'yaml' package is not installed. Please install it by running 'pip install pyyaml'")
    exit(1)

# Load configuration settings
CONFIG_PATH = "drone_config.yaml"
if os.path.exists(CONFIG_PATH):
    with open(CONFIG_PATH, 'r') as file:
        config = yaml.safe_load(file)
else:
    # Default settings if config file is not present
    config = {
        'port': 'COM3',
        'baudrate': 9600,
        'default_altitude': 50,
        'safety_altitude': 20
    }

# Enable logging to a file for easier debugging
logging.basicConfig(filename='drone_controller.log', level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

# Initialize OpenAI API
api_key = "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"
if not api_key:
    raise openai.error.AuthenticationError(
        "No API key provided. You can set your API key in code using 'openai.api_key = <API-KEY>', or you can set the environment variable OPENAI_API_KEY=<API-KEY>). If your API key is stored in a file, you can point the openai module at it with 'openai.api_key_path = <PATH>'. You can generate API keys in the OpenAI web interface. See https://platform.openai.com/account/api-keys for details."
    )
openai.api_key = api_key

class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.base_Kp = Kp
        self.base_Ki = Ki
        self.base_Kd = Kd
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.integral = 0
        self.previous_error = 0
        self.last_time = time.time()

    def update(self, current_value):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0.0:
            dt = 1e-16  # Avoid division by zero

        error = self.setpoint - current_value
        self.adjust_gains(error, dt)

        self.integral += error * dt
        derivative = (error - self.previous_error) / dt

        control_value = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        self.previous_error = error
        self.last_time = current_time

        return control_value

    def adjust_gains(self, error, dt):
        error_magnitude = abs(error)
        derivative = (error - self.previous_error) / dt
        derivative_magnitude = abs(derivative)

        self.Kp = self.base_Kp * (1 + 0.5 * error_magnitude / max(self.setpoint, 1))
        self.Kd = self.base_Kd * (1 + 0.5 * derivative_magnitude / max(error_magnitude, 1))

        self.Kp = max(0.1 * self.base_Kp, min(10 * self.base_Kp, self.Kp))
        self.Kd = max(0.1 * self.base_Kd, min(10 * self.base_Kd, self.Kd))

class DroneController:
    def __init__(self):
        self.port = config['port']
        self.baudrate = config['baudrate']
        self.arduino = None
        self.landing_in_progress = False
        self.in_flight = False
        self.safety_altitude = config['safety_altitude']

        self.pid_controller = PIDController(Kp=1.0, Ki=0.0, Kd=0.1)
        self.desired_altitude = 0
        self.current_altitude = 0

        self.command_history = []
        self.command_queue = queue.Queue()
        self.last_ai_suggestion = ''
        self.command_index = None  # For navigating command history
        self.shortcuts = {}  # For custom shortcuts
        self.offline_command_queue = []  # For offline mode
        self.connected = False

        self.executor = ThreadPoolExecutor()
        self.altitude_shutdown_event = threading.Event()

        self.run_gui_interface()
        self.start_altitude_thread()

    def connect_to_drone(self):
        try:
            self.arduino = serial.Serial(self.port, self.baudrate, timeout=1)
            self.update_status("Connected")
            self.log_message("Connected to drone.")
            self.connected = True
            self.process_offline_commands()
        except serial.SerialException as e:
            self.log_message(f"Connection failed: {e}")
            self.connected = False
            raise e

    def disconnect_drone(self):
        if self.arduino:
            self.arduino.close()
            self.arduino = None
            self.in_flight = False
            self.landing_in_progress = False
            self.connected = False
            self.altitude_shutdown_event.set()  # Stop altitude control thread
            self.update_status("Disconnected")
            self.log_message("Disconnected from drone.")

    def send_command(self, command, message):
        try:
            if self.arduino:
                self.arduino.write(command.encode('utf-8'))
            self.log_message(message)
        except Exception as e:
            self.log_message(f"Error: {e}")
            logging.error(f"Error sending command: {e}")

    def takeoff(self):
        if not self.landing_in_progress:
            self.in_flight = True
            if self.connected:
                self.send_command('TAKEOFF\n', "Takeoff initiated")
                self.update_progress("Takeoff in progress")
            else:
                self.queue_offline_command('TAKEOFF', "Takeoff queued")
            self.update_status("In Flight")

    def land(self):
        if self.in_flight:
            self.landing_in_progress = True
            if self.connected:
                self.send_command('LAND\n', "Landing initiated")
                self.update_progress("Landing in progress")
            else:
                self.queue_offline_command('LAND', "Landing queued")
            self.in_flight = False
            self.update_status("Landing")

    def update_altitude(self, altitude):
        self.desired_altitude = altitude
        self.pid_controller.setpoint = altitude
        if self.connected:
            self.log_message(f"Altitude set to {altitude} cm")
            self.update_progress(f"Setting altitude to {altitude} cm")
        else:
            self.queue_offline_command(f'SET_ALTITUDE {altitude}', f"Altitude set to {altitude} cm queued")

    def check_battery_status(self):
        battery_status = random.randint(0, 100)
        self.log_message(f"Battery status: {battery_status}%")
        self.update_battery_gauge(battery_status)
        self.update_progress("Battery status checked")

    def calibrate_sensors(self):
        self.log_message("Calibrating sensors...")
        self.update_progress("Calibrating sensors")
        time.sleep(0.5)  # Reduced calibration time for speed
        self.log_message("Sensors calibrated successfully.")
        self.update_progress("Calibration complete")

    def log_status(self):
        log_entry = f"Altitude: {self.current_altitude} cm, Battery: {random.randint(0, 100)}%, Speed: {random.randint(0, 1000)} RPM"
        self.log_message(f"Status logged: {log_entry}")
        with open('drone_status_log.txt', 'a') as log_file:
            log_file.write(f"{time.ctime()}: {log_entry}\n")
        self.update_progress("Status logged")

    def set_condition(self, condition):
        self.command_queue.put(condition)
        self.log_message(f"Condition set: {condition}")
        self.update_progress(f"Condition set: {condition}")

    def start_altitude_thread(self):
        threading.Thread(target=self.altitude_control_loop, daemon=True).start()

    def altitude_control_loop(self):
        while not self.altitude_shutdown_event.is_set():
            if self.in_flight and self.connected:
                control_signal = self.pid_controller.update(self.current_altitude)
                self.send_command(f'SET_THRUST {control_signal}\n', f"Adjusting thrust: {control_signal}")
            time.sleep(0.05)  # Increased loop frequency for faster response

    def run_gui_interface(self):
        self.root = tk.Tk()
        self.root.title("Drone Command Interface")
        self.root.geometry("1000x700")
        self.root.configure(bg="#0F0F0F")

        self.terminal = scrolledtext.ScrolledText(self.root, wrap=tk.WORD, width=70, height=15, bg="#0F0F0F", fg="#00FF00", font=("Courier", 12), insertbackground="#00FF00")
        self.terminal.pack(pady=10)
        self.terminal.bind("<Return>", self.process_terminal_command)
        self.terminal.bind("<Tab>", self.auto_complete_command)
        self.root.bind("<Up>", self.previous_command)
        self.root.bind("<Down>", self.next_command)
        self.root.bind("<Right>", self.insert_ai_suggestion)

        self.status_label = tk.Label(self.root, text="Status: Idle", bg="#0F0F0F", fg="#00FF00", font=("Courier", 12))
        self.status_label.pack(pady=5)

        self.battery_gauge = Canvas(self.root, width=200, height=200, bg="#0F0F0F", highlightthickness=0)
        self.battery_gauge.pack(side=tk.LEFT, padx=20)
        self.update_battery_gauge(100)

        self.rotor_speed_gauge = Canvas(self.root, width=200, height=200, bg="#0F0F0F", highlightthickness=0)
        self.rotor_speed_gauge.pack(side=tk.RIGHT, padx=20)
        self.update_rotor_speed_gauge(0)

        self.altitude_label = tk.Label(self.root, text="Altitude: 0 cm", bg="#0F0F0F", fg="#00FF00", font=("Courier", 12))
        self.altitude_label.pack(pady=5)

        self.progress_label = tk.Label(self.root, text="Progress: Idle", bg="#0F0F0F", fg="#00FF00", font=("Courier", 12))
        self.progress_label.pack(pady=5)

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()

    def process_terminal_command(self, event):
        command = self.terminal.get("end-2l", "end-1c").strip()
        if command:
            self.terminal.insert(tk.END, "\n")
            self.command_history.append(command)
            self.command_index = None

            if command.startswith('/alias '):
                try:
                    alias, original_command = command.split(' ')[1:]
                    self.aliases[alias] = original_command
                    self.log_message(f"Alias set: {alias} -> {original_command}")
                except ValueError:
                    self.log_message("Invalid alias format. Usage: /alias <alias> <command>")
            elif command.startswith('/bind '):
                try:
                    shortcut, original_command = command.split(' ')[1:]
                    self.shortcuts[shortcut] = original_command
                    self.log_message(f"Shortcut set: {shortcut} -> {original_command}")
                except ValueError:
                    self.log_message("Invalid bind format. Usage: /bind <key> <command>")
            elif command in self.shortcuts:
                self.process_terminal_command_internal(self.shortcuts[command])
            elif command == '/help':
                self.display_help()
            elif command == '/takeoff':
                self.takeoff()
            elif command == '/land':
                self.land()
            elif command.startswith('/altitude '):
                try:
                    altitude = int(command.split()[1])
                    self.update_altitude(altitude)
                except (IndexError, ValueError):
                    self.log_message("Invalid altitude value. Usage: /altitude <value>")
            elif command == '/connect':
                self.connect_to_drone()
            elif command == '/disconnect':
                self.disconnect_drone()
            elif command.startswith('/condition '):
                try:
                    condition = command.split(' ', 1)[1]
                    self.set_condition(condition)
                except IndexError:
                    self.log_message("Usage: /condition <condition>")
            elif command == '/battery_status':
                self.check_battery_status()
            elif command == '/calibrate':
                self.calibrate_sensors()
            elif command == '/log_status':
                self.log_status()
            elif command == '/ai_suggest':
                suggestion = self.ai_suggest_command()
                self.last_ai_suggestion = suggestion
                self.log_message(f"AI Suggestion: {suggestion}")
            elif command == '/reset':
                self.reset_terminal()
                self.log_message("Application restarting...")
                self.root.destroy()
                python = sys.executable
                os.execl(python, python, *sys.argv)
            elif command == '/clear':
                self.clear_terminal()
            elif command == '/emergency_land':
                self.emergency_land()
            elif command == '/set_safety_altitude':
                self.set_safety_altitude()
            elif command == '/ai_assist':
                self.ai_assist(command)
            elif command == '/ai_optimize':
                self.ai_optimize_flight()
            elif command == '/ai_predict_battery':
                self.ai_predict_battery_life()
            elif command == '/ai_health_check':
                self.ai_health_check()
            else:
                self.log_message(f"Unknown command: {command}")

        self.terminal.insert(tk.END, "\n")
        self.lock_cursor_position()

    def ai_assist(self, command):
        # Extract the specific command user wants assistance with
        command_to_assist = command.replace('/ai_assist', '').strip()
        if not command_to_assist:
            self.log_message("Please specify a command to get assistance with.")
            return
    
        prompt = f"Assist with command: {command_to_assist}"
    
        # Run the OpenAI request in a separate thread
        self.executor.submit(self.get_ai_assistance, prompt)

    def get_ai_assistance(self, prompt):
        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": "You are an assistant helping with drone commands."},
                    {"role": "user", "content": prompt}
                ],
                max_tokens=100
            )
            suggestion = response.choices[0].message['content'].strip()
            self.log_message(f"AI Assistance: {suggestion}")
        except Exception as e:
            self.log_message(f"Error fetching AI assistance: {e}")
            logging.error(f"OpenAI API error: {e}")

    def ai_optimize_flight(self):
        prompt = "Optimize the flight parameters for a drone to achieve maximum efficiency and stability."
        response = openai.Completion.create(
            engine="text-davinci-003",
            prompt=prompt,
            max_tokens=150
        )
        optimization_suggestions = response.choices[0].text.strip()
        self.log_message(f"Flight Optimization Suggestions: {optimization_suggestions}")
        self.update_progress("Flight optimization suggestions provided")

    def ai_predict_battery_life(self):
        prompt = "Predict the remaining battery life for a drone given current usage patterns."
        response = openai.Completion.create(
            engine="text-davinci-003",
            prompt=prompt,
            max_tokens=50
        )
        prediction = response.choices[0].text.strip()
        self.log_message(f"Battery Life Prediction: {prediction}")
        self.update_progress("Battery life prediction provided")

    def ai_health_check(self):
        prompt = "Perform a health check on all drone systems including sensors, motors, and battery."
        response = openai.Completion.create(
            engine="text-davinci-003",
            prompt=prompt,
            max_tokens=100
        )
        health_check_report = response.choices[0].text.strip()
        self.log_message(f"Health Check Report: {health_check_report}")
        self.update_progress("Health check completed")

    def process_terminal_command_internal(self, command):
        self.terminal.insert(tk.END, f"{command}\n")
        self.process_terminal_command(None)

    def previous_command(self, event):
        if self.command_history:
            if self.command_index is None:
                self.command_index = len(self.command_history)
            if self.command_index > 0:
                self.command_index -= 1
                previous_command = self.command_history[self.command_index]
                self.terminal.delete("end-1l", "end-1c")
                self.terminal.insert(tk.END, previous_command)
                self.lock_cursor_position()

    def next_command(self, event):
        if self.command_history:
            if self.command_index is not None and self.command_index < len(self.command_history) - 1:
                self.command_index += 1
                next_command = self.command_history[self.command_index]
                self.terminal.delete("end-1l", "end-1c")
                self.terminal.insert(tk.END, next_command)
            else:
                self.command_index = len(self.command_history)
                self.terminal.delete("end-1l", "end-1c")
            self.lock_cursor_position()

    def insert_ai_suggestion(self, event):
        if self.last_ai_suggestion:
            # Clear current input line and insert the AI suggestion
            self.terminal.delete("end-2l", "end-1l")
            self.terminal.insert(tk.END, self.last_ai_suggestion + "\n")
            self.lock_cursor_position()

    def auto_complete_command(self, event):
        command_prefix = self.terminal.get("end-2l", "end-1c").strip()
        matching_commands = [cmd for cmd in ["/takeoff", "/land", "/altitude", "/connect", "/disconnect", "/condition", "/battery_status", "/calibrate", "/log_status", "/reset", "/clear", "/ai_suggest", "/set_safety_altitude", "/emergency_land", "/ai_assist", "/ai_optimize", "/ai_predict_battery", "/ai_health_check"] if cmd.startswith(command_prefix)]
        if len(matching_commands) == 1:
            self.terminal.delete("end-2l", "end-1c")
            self.terminal.insert(tk.END, matching_commands[0])
        return "break"

    def lock_cursor_position(self):
        self.terminal.mark_set(tk.INSERT, "end-1c")
        self.terminal.see(tk.INSERT)

    def reset_terminal(self):
        self.terminal.delete(1.0, tk.END)
        self.command_history = []
        self.command_index = None
        self.log_message("Terminal has been reset.")
        self.update_progress("Terminal reset")

    def clear_terminal(self):
        self.terminal.delete(1.0, tk.END)
        self.log_message("Terminal output cleared.")
        self.update_progress("Terminal output cleared")

    def ai_suggest_command(self):
        suggestions = ["/takeoff", "/land", "/altitude 100", "/connect", "/disconnect", "/ai_optimize", "/ai_predict_battery", "/ai_health_check"]
        return random.choice(suggestions)

    def display_help(self):
        help_text = """
Available commands:
/help                  - Show this help message
/takeoff               - Initiate drone takeoff
/land                  - Initiate drone landing
/altitude <cm>         - Set desired altitude in cm
/connect               - Connect to the drone
/disconnect            - Disconnect from the drone and exit
/condition <c>         - Set a condition for automatic actions (e.g., 'land if battery < 20%')
/battery_status        - Get the current battery status
/calibrate             - Calibrate the drone's sensors
/log_status            - Log the current status (altitude, speed, battery)
/ai_suggest            - Get an AI-generated command suggestion
/reset                 - Reset the terminal and restart the application
/clear                 - Clear the terminal output without resetting history
/alias <alias> <cmd>   - Set a command alias (e.g., /alias t /takeoff)
/bind <key> <cmd>      - Bind a shortcut key to a command
/set_safety_altitude   - Set safety altitude for emergency landings
/emergency_land        - Force emergency landing
/ai_assist             - Get AI assistance for a specific command
/ai_optimize           - Optimize flight parameters for maximum efficiency
/ai_predict_battery    - Predict remaining battery life
/ai_health_check       - Perform a health check on all drone systems
        """
        self.log_message(help_text)
        self.update_progress("Help displayed")

    def log_message(self, message):
        self.terminal.insert(tk.END, f"{message}\n")
        self.terminal.see(tk.END)
        logging.info(message)

    def update_status(self, status):
        self.status_label.config(text=f"Status: {status}")

    def update_progress(self, progress):
        self.progress_label.config(text=f"Progress: {progress}")

    def on_closing(self):
        self.disconnect_drone()
        self.generate_mission_report()
        self.root.destroy()

    def generate_mission_report(self):
        report = "Mission Report:\n"
        report += f"Commands Executed: {len(self.command_history)}\n"
        report += "\n".join(self.command_history)
        report += "\nMission Completed Successfully."
        with open('mission_report.txt', 'w') as report_file:
            report_file.write(report)
        self.log_message("Mission report generated.")
        self.update_progress("Mission report generated")

    def update_battery_gauge(self, percentage):
        self.battery_gauge.delete("all")
        self.battery_gauge.create_oval(10, 10, 190, 190, outline="#00FF00", width=4)
        self.battery_gauge.create_arc(10, 10, 190, 190, start=90, extent=-percentage * 3.6, outline="#00FF00", style="arc", width=8)
        self.battery_gauge.create_text(100, 100, text=f"{percentage}%", fill="#00FF00", font=("Courier", 14))

    def update_rotor_speed_gauge(self, speed):
        self.rotor_speed_gauge.delete("all")
        self.rotor_speed_gauge.create_oval(10, 10, 190, 190, outline="#00FF00", width=4)
        self.rotor_speed_gauge.create_arc(10, 10, 190, 190, start=90, extent=-speed * 3.6, outline="#00FF00", style="arc", width=8)
        self.rotor_speed_gauge.create_text(100, 100, text=f"{speed} RPM", fill="#00FF00", font=("Courier", 14))

    def queue_offline_command(self, command, message):
        self.offline_command_queue.append(command)
        self.log_message(message)
        self.update_progress(f"Command queued: {command}")

    def process_offline_commands(self):
        while self.offline_command_queue:
            command = self.offline_command_queue.pop(0)
            self.send_command(command + '\n', f"Executed queued command: {command}")
        self.update_progress("Offline commands processed")

    def set_safety_altitude(self):
        self.safety_altitude = config['safety_altitude']
        self.log_message(f"Safety altitude set to {self.safety_altitude} cm")
        self.update_progress(f"Safety altitude set to {self.safety_altitude} cm")

    def emergency_land(self):
        self.log_message("Emergency landing initiated!")
        self.send_command('LAND\n', "Emergency landing initiated")
        self.in_flight = False
        self.update_status("Emergency Landing")
        self.update_progress("Emergency landing in progress")

if __name__ == "__main__":
    try:
        DroneController()
    except KeyboardInterrupt:
        print("\nExiting...")
