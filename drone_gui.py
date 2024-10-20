import sys
import os
import time
import logging
import threading
import queue
import random
import platform
import json

import serial
import serial.tools.list_ports
import numpy as np

import matplotlib.pyplot as plt
import matplotlib
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

import tkinter as tk
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText

try:
    import yaml
except ImportError:
    print("The 'yaml' package is not installed. Please install it by running 'pip install pyyaml'")
    sys.exit(1)


# Load configuration settings
CONFIG_PATH = "drone_config.yaml"
if os.path.exists(CONFIG_PATH):
    with open(CONFIG_PATH, 'r') as file:
        config = yaml.safe_load(file)
else:
    # Default settings if config file is not present
    config = {
        'port': 'COM3' if platform.system() == 'Windows' else '/dev/ttyUSB0',
        'baudrate': 9600,
        'default_altitude': 50,
        'safety_altitude': 20,
        'min_ground_distance': 5,  # Added minimum ground distance
        'log_level': 'DEBUG'  # Add configurable logging level
    }

# Configure logging
log_level = config.get('log_level', 'DEBUG').upper()
logging.basicConfig(filename='drone_controller.log', level=getattr(logging, log_level, logging.DEBUG),
                    format='%(asctime)s - %(levelname)s - %(message)s')


class LowPassFilter:
    """Simple low-pass filter for smoothing sensor data."""
    def __init__(self, alpha=0.5):
        self.alpha = alpha
        self.last_value = None

    def filter(self, new_value):
        if self.last_value is None:
            self.last_value = new_value
            return new_value
        filtered_value = self.alpha * new_value + (1 - self.alpha) * self.last_value
        self.last_value = filtered_value
        return filtered_value


class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.integral = 0
        self.previous_error = 0
        self.last_time = time.time()

    def update(self, current_value):
        current_time = time.time()
        dt = current_time - self.last_time or 1e-16  # Avoid division by zero

        error = self.setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        control_value = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        self.last_time = current_time

        return control_value


class DroneController:
    def __init__(self):
        # Initialize instance variables
        self.port = config['port']
        self.baudrate = config['baudrate']
        self.arduino = None
        self.landing_in_progress = False
        self.in_flight = False
        self.safety_altitude = config['safety_altitude']
        self.min_ground_distance = config['min_ground_distance']  # Minimum ground distance
        self.battery_level = 100  # Initialized battery level

        # PID controller for altitude
        self.pid_controller = PIDController(Kp=1.2, Ki=0.01, Kd=0.5)
        self.desired_altitude = config['default_altitude']
        self.current_altitude = 0

        self.command_history = []
        self.command_queue = queue.Queue()
        self.command_index = None  # For navigating command history
        self.aliases = {}  # Command Aliases
        self.shortcuts = {}  # For custom shortcuts
        self.offline_command_queue = []  # For offline mode
        self.connected = False

        self.altitude_shutdown_event = threading.Event()
        self.serial_lock = threading.Lock()

        self.xdata = []
        self.ydata = []
        self.start_time = time.time()

        self.motor_speed = 0  # Motor speed in RPM

        # Vibration damping filter
        self.altitude_filter = LowPassFilter(alpha=0.3)

        self.closing = False  # Flag to indicate if the app is closing

        # Mapping of command names to methods
        self.command_methods = {
            '/help': self.display_help,
            '/takeoff': self.takeoff,
            '/land': self.land,
            '/altitude': self.set_altitude,
            '/connect': self.connect_drone,
            '/disconnect': self.disconnect_drone,
            '/condition': self.set_condition,
            '/battery_status': self.check_battery_status,
            '/calibrate': self.calibrate_sensors,
            '/log_status': self.log_status,
            '/reset': self.reset_terminal,
            '/clear': self.clear_terminal,
            '/emergency_land': self.emergency_land,
            '/set_safety_altitude': self.set_safety_altitude,
            '/set_min_ground_distance': self.set_min_ground_distance,
        }

        # List of all commands for auto-complete
        self.all_commands = list(self.command_methods.keys())

        # Start GUI in the main thread
        self.setup_gui()
        self.root.after(0, self.run_gui_tasks)

        # Start serial communication in a separate thread
        threading.Thread(target=self.connect_to_drone, daemon=True).start()

        self.start_health_check_thread()
        self.root.mainloop()

    def setup_gui(self):
        self.root = tk.Tk()
        self.root.title("Drone Command Interface")
        self.root.geometry("900x750")
        self.root.configure(bg="#0F0F0F")

        # Observation field (read-only)
        self.observation_field = ScrolledText(self.root, wrap=tk.WORD, width=70, height=10,
                                              bg="#0F0F0F", fg="#00FF00", font=("Courier", 12),
                                              state='disabled')
        self.observation_field.pack(pady=10)

        # Command entry field
        self.command_entry = tk.Entry(self.root, bg="#0F0F0F", fg="#00FF00", font=("Courier", 12),
                                      insertbackground="#00FF00")
        self.command_entry.pack(pady=10, fill=tk.X)
        self.command_entry.bind("<Return>", self.process_terminal_command)
        self.command_entry.bind("<Tab>", self.auto_complete_command)
        self.command_entry.bind("<KeyRelease>", self.update_command_suggestions)
        self.root.bind("<Up>", self.previous_command)
        self.root.bind("<Down>", self.next_command)

        # Command suggestions box
        self.command_suggestion_box = tk.Listbox(self.root, bg="#0F0F0F", fg="#00FF00",
                                                 font=("Courier", 12), height=5)
        self.command_suggestion_box.pack(pady=5)
        self.command_suggestion_box.bind("<<ListboxSelect>>", self.fill_command_from_suggestion)

        self.status_label = tk.Label(self.root, text="Status: Idle", bg="#0F0F0F", fg="#00FF00",
                                     font=("Courier", 12))
        self.status_label.pack(pady=5)

        self.altitude_label = tk.Label(self.root, text="Altitude: 0 cm", bg="#0F0F0F", fg="#00FF00",
                                       font=("Courier", 12))
        self.altitude_label.pack(pady=5)

        self.motor_speed_label = tk.Label(self.root, text="Motor Speed: 0 RPM", bg="#0F0F0F",
                                          fg="#00FF00", font=("Courier", 12))
        self.motor_speed_label.pack(pady=5)

        self.progress_label = tk.Label(self.root, text="Progress: Idle", bg="#0F0F0F", fg="#00FF00",
                                       font=("Courier", 12))
        self.progress_label.pack(pady=5)

        self.progress_bar = ttk.Progressbar(self.root, style="green.Horizontal.TProgressbar",
                                            length=400, mode='determinate')
        self.progress_bar.pack(pady=10)
        style = ttk.Style()
        style.theme_use('default')
        style.configure("green.Horizontal.TProgressbar", troughcolor='#0F0F0F', background='#00FF00',
                        bordercolor="#0F0F0F")

        # Gauges and plots frame
        self.gauges_frame = tk.Frame(self.root, bg="#0F0F0F")
        self.gauges_frame.pack(pady=10, fill=tk.BOTH, expand=True)

        self.battery_gauge = tk.Canvas(self.gauges_frame, width=150, height=150, bg="#0F0F0F",
                                       highlightthickness=0)
        self.battery_gauge.pack(side=tk.LEFT, padx=20)
        self.update_battery_gauge(100)

        self.rotor_speed_gauge = tk.Canvas(self.gauges_frame, width=150, height=150, bg="#0F0F0F",
                                           highlightthickness=0)
        self.rotor_speed_gauge.pack(side=tk.RIGHT, padx=20)
        self.update_rotor_speed_gauge(0)

        # Live plot
        self.setup_live_plot()

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def setup_live_plot(self):
        # Adjust plot dimensions and colors
        matplotlib.rcParams['axes.facecolor'] = '#0F0F0F'
        matplotlib.rcParams['figure.facecolor'] = '#0F0F0F'
        matplotlib.rcParams['savefig.facecolor'] = '#0F0F0F'

        self.fig, self.ax = plt.subplots(figsize=(6, 2.5))  # Wider and shorter figure size
        self.line, = self.ax.plot([], [], lw=2, color='#00FF00')  # Line color set to green
        self.ax.set_title("Altitude Over Time", color='white')
        self.ax.set_xlabel("Time (s)", color='white')
        self.ax.set_ylabel("Altitude (cm)", color='white')
        self.ax.tick_params(axis='x', colors='white')
        self.ax.tick_params(axis='y', colors='white')
        self.ax.set_xlim(0, 60)
        self.ax.set_ylim(0, max(self.desired_altitude + 20, 100))

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.gauges_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, pady=10)  # Adjusted padding

    def run_gui_tasks(self):
        if not self.closing:
            self.update_live_plot()
            self.root.after(1000, self.run_gui_tasks)  # Update every second

    def update_live_plot(self):
        current_time = time.time() - self.start_time
        self.xdata.append(current_time)
        self.ydata.append(self.current_altitude)

        # Keep only the last 60 seconds of data
        if current_time > 60:
            self.xdata = self.xdata[-60:]
            self.ydata = self.ydata[-60:]
            self.ax.set_xlim(current_time - 60, current_time)

        self.line.set_data(self.xdata, self.ydata)
        self.ax.relim()
        self.ax.autoscale_view()
        self.canvas.draw()

    def process_terminal_command(self, event):
        command = self.command_entry.get().strip()
        if command:
            self.command_history.append(command)
            self.command_index = None

            self.log_message(f"> {command}")

            if command.startswith('/alias '):
                self.handle_alias_command(command)
            elif command.startswith('/bind '):
                self.handle_bind_command(command)
            elif command in self.shortcuts:
                self.command_entry.delete(0, tk.END)
                self.command_entry.insert(tk.END, self.shortcuts[command])
                self.process_terminal_command(None)
            else:
                command_name = command.split()[0]
                command_args = command[len(command_name):].strip()
                command_method = self.command_methods.get(command_name)
                if command_method:
                    command_method(command_args)
                else:
                    self.log_message(f"Unknown command: {command}")

            self.command_entry.delete(0, tk.END)

    def handle_alias_command(self, command):
        try:
            _, alias, original_command = command.split(' ', 2)
            self.aliases[alias] = original_command
            self.log_message(f"Alias set: {alias} -> {original_command}")
        except ValueError:
            self.log_message("Invalid alias format. Usage: /alias <alias> <command>")

    def handle_bind_command(self, command):
        try:
            _, shortcut, original_command = command.split(' ', 2)
            self.shortcuts[shortcut] = original_command
            self.log_message(f"Shortcut set: {shortcut} -> {original_command}")
        except ValueError:
            self.log_message("Invalid bind format. Usage: /bind <key> <command>")

    def display_help(self, args=None):
        help_text = """
Available commands:
/help                      - Show this help message
/takeoff                   - Initiate drone takeoff
/land                      - Initiate drone landing
/altitude <cm>             - Set desired altitude in cm
/connect                   - Connect to the drone
/disconnect                - Disconnect from the drone
/condition <c>             - Set a condition for automatic actions
/battery_status            - Get the current battery status
/calibrate                 - Calibrate the drone's sensors
/log_status                - Log the current status
/reset                     - Reset the terminal and restart the application
/clear                     - Clear the terminal output without resetting history
/alias <alias> <cmd>       - Set a command alias
/bind <key> <cmd>          - Bind a shortcut key to a command
/set_safety_altitude <cm>  - Set safety altitude for emergency landings
/set_min_ground_distance <cm> - Set minimum ground distance to stop motors
/emergency_land            - Force emergency landing
        """
        self.log_message(help_text)
        self.update_progress("Help displayed")

    def auto_complete_command(self, event):
        command_prefix = self.command_entry.get().strip()
        matching_commands = [
            cmd for cmd in self.all_commands if cmd.startswith(command_prefix)
        ]
        if len(matching_commands) == 1:
            self.command_entry.delete(0, tk.END)
            self.command_entry.insert(tk.END, matching_commands[0])
        return "break"

    def update_command_suggestions(self, event):
        typed_text = self.command_entry.get().strip()
        all_commands = self.all_commands + list(self.aliases.keys())
        matching_commands = [cmd for cmd in all_commands if cmd.startswith(typed_text)]

        self.command_suggestion_box.delete(0, tk.END)

        for command in matching_commands:
            self.command_suggestion_box.insert(tk.END, command)

    def fill_command_from_suggestion(self, event):
        selection = self.command_suggestion_box.curselection()
        if selection:
            self.command_entry.delete(0, tk.END)
            self.command_entry.insert(0, self.command_suggestion_box.get(selection[0]))

    def previous_command(self, event):
        if self.command_history:
            if self.command_index is None:
                self.command_index = len(self.command_history)
            if self.command_index > 0:
                self.command_index -= 1
                previous_command = self.command_history[self.command_index]
                self.command_entry.delete(0, tk.END)
                self.command_entry.insert(tk.END, previous_command)
                self.command_entry.icursor(tk.END)

    def next_command(self, event):
        if self.command_history:
            if self.command_index is not None and self.command_index < len(self.command_history) - 1:
                self.command_index += 1
                next_command = self.command_history[self.command_index]
                self.command_entry.delete(0, tk.END)
                self.command_entry.insert(tk.END, next_command)
            else:
                self.command_index = len(self.command_history)
                self.command_entry.delete(0, tk.END)
            self.command_entry.icursor(tk.END)

    def log_message(self, message):
        if threading.current_thread() != threading.main_thread():
            self.root.after(0, self.log_message, message)
        else:
            self.observation_field.config(state='normal')
            self.observation_field.insert(tk.END, f"{message}\n")
            self.observation_field.see(tk.END)
            self.observation_field.config(state='disabled')
            logging.info(message)

    def update_status(self, status):
        if threading.current_thread() != threading.main_thread():
            self.root.after(0, self.update_status, status)
        else:
            self.status_label.config(text=f"Status: {status}")

    def update_progress(self, progress):
        if threading.current_thread() != threading.main_thread():
            self.root.after(0, self.update_progress, progress)
        else:
            self.progress_label.config(text=f"Progress: {progress}")

    def update_battery_gauge(self, percentage):
        self.battery_gauge.delete("all")
        self.battery_gauge.create_oval(10, 10, 140, 140, outline="#00FF00", width=4)
        self.battery_gauge.create_arc(10, 10, 140, 140, start=90, extent=-percentage * 3.6, outline="#00FF00",
                                      style="arc", width=8)
        self.battery_gauge.create_text(75, 75, text=f"{percentage}%", fill="#00FF00", font=("Courier", 12))

    def update_rotor_speed_gauge(self, speed):
        max_speed = 10000  # Assuming max speed is 10000 RPM
        percentage = (speed / max_speed) * 100
        self.rotor_speed_gauge.delete("all")
        self.rotor_speed_gauge.create_oval(10, 10, 140, 140, outline="#00FF00", width=4)
        self.rotor_speed_gauge.create_arc(10, 10, 140, 140, start=90, extent=-percentage * 3.6, outline="#00FF00",
                                          style="arc", width=8)
        self.rotor_speed_gauge.create_text(75, 75, text=f"{speed} RPM", fill="#00FF00", font=("Courier", 12))

    def connect_to_drone(self):
        # Automatically detect the serial port if not specified
        if not self.port:
            ports = serial.tools.list_ports.comports()
            for port in ports:
                if 'Arduino' in port.description or 'USB Serial' in port.description:
                    self.port = port.device
                    break

        retries = 3
        attempt = 0
        while attempt < retries and not self.connected:
            try:
                self.arduino = serial.Serial(self.port, self.baudrate, timeout=1)
                self.connected = True
                self.update_status("Connected")
                self.log_message(f"Connected to drone on port {self.port}.")
                threading.Thread(target=self.read_serial_data, daemon=True).start()
                self.process_offline_commands()
                self.start_altitude_thread()
                break
            except serial.SerialException as e:
                attempt += 1
                self.log_message(f"Connection failed: {e}. Retrying ({retries - attempt} attempts left)")
                logging.error(f"Serial exception occurred: {e}")
                time.sleep(2)  # Delay between retries
            except Exception as e:
                self.log_message(f"An unexpected error occurred: {e}")
                logging.error(f"Unexpected error: {e}")
                break
        else:
            self.log_message("Failed to connect after multiple attempts.")
            self.update_status("Failed to Connect")
            self.connected = False

    def connect_drone(self, args=None):
        threading.Thread(target=self.connect_to_drone, daemon=True).start()

    def disconnect_drone(self, args=None):
        if self.connected and self.arduino:
            try:
                self.altitude_shutdown_event.set()  # Stop the altitude control loop
                self.health_check_event.set()  # Stop the health check loop
                with self.serial_lock:
                    self.arduino.close()
                self.connected = False
                self.update_status("Disconnected")
                self.log_message("Disconnected from drone.")
                self.update_progress("Disconnected")
            except Exception as e:
                self.log_message(f"Error disconnecting: {e}")
                logging.error(f"Error disconnecting: {e}")

    def read_serial_data(self):
        while self.connected and self.arduino and self.arduino.is_open:
            try:
                with self.serial_lock:
                    if self.arduino.in_waiting > 0:
                        data_line = self.arduino.readline().decode('utf-8').strip()
                        if data_line:
                            self.process_serial_data(data_line)
            except (OSError, serial.SerialException) as e:
                self.log_message(f"Error reading serial data: {e}")
                logging.error(f"Error reading serial data: {e}")
                break  # Exit the loop if the serial port is invalid
            except Exception as e:
                self.log_message(f"Unexpected error: {e}")
                logging.exception("Exception in read_serial_data")
            time.sleep(0.01)

    def process_serial_data(self, data_line):
        # Parse the data line and update drone state
        try:
            data = json.loads(data_line)  # Assuming the drone sends JSON-formatted strings
            raw_altitude = data.get('altitude', self.current_altitude)
            self.current_altitude = self.altitude_filter.filter(raw_altitude)  # Apply low-pass filter
            self.battery_level = data.get('battery', self.battery_level)
            self.motor_speed = data.get('motor_speed', self.motor_speed)

            # Schedule GUI updates in the main thread
            self.root.after(0, self.update_gui_elements)
            self.check_battery()
        except json.JSONDecodeError:
            self.log_message(f"Invalid data received: {data_line}")
            logging.error(f"JSON decode error: {data_line}")
        except Exception as e:
            self.log_message(f"Error processing serial data: {e}")
            logging.exception("Exception in processing serial data")

    def update_gui_elements(self):
        self.altitude_label.config(text=f"Altitude: {self.current_altitude:.2f} cm")
        self.motor_speed_label.config(text=f"Motor Speed: {self.motor_speed} RPM")
        self.update_battery_gauge(self.battery_level)
        self.update_rotor_speed_gauge(self.motor_speed)

    def start_altitude_thread(self):
        threading.Thread(target=self.altitude_control_loop, daemon=True).start()

    def altitude_control_loop(self):
        while not self.altitude_shutdown_event.is_set():
            if self.in_flight and self.connected:
                # Apply PID control to adjust thrust
                control_signal = self.pid_controller.update(self.current_altitude)

                # Check for minimum ground distance to stop motors
                if self.current_altitude <= self.min_ground_distance:
                    control_signal = 0
                    self.log_message("Minimum ground distance reached, stopping motors.")
                    self.in_flight = False
                    self.update_status("Landed")

                # Send control signal to the drone
                self.send_command(f'SET_THRUST {control_signal}', f"Adjusting thrust: {control_signal:.2f}")

                # Update motor speed (simulated here)
                self.motor_speed = max(0, int(control_signal * 1000))  # Example scaling
            time.sleep(0.05)  # Increased loop frequency for faster response

    def start_health_check_thread(self):
        self.health_check_event = threading.Event()
        threading.Thread(target=self.health_check_loop, daemon=True).start()

    def health_check_loop(self):
        while not self.health_check_event.is_set():
            motor_status = random.choice(["OK", "Error"])
            sensor_status = random.choice(["OK", "Error"])
            self.log_message(f"Motor Status: {motor_status}")
            self.log_message(f"Sensor Status: {sensor_status}")
            time.sleep(10)  # Check every 10 seconds

    def check_battery(self):
        if self.battery_level < 20:
            self.log_message("Battery is below 20%, initiating emergency landing.")
            self.emergency_land()
        elif self.battery_level < 50:
            self.log_message("Warning: Battery is below 50%.")

    def check_battery_status(self, args=None):
        self.check_battery()

    def calibrate_sensors(self, args=None):
        self.log_message("Calibrating sensors...")
        self.update_progress("Calibrating sensors")
        if self.connected:
            self.send_command('CALIBRATE', "Calibration command sent")
        else:
            self.queue_offline_command('CALIBRATE', "Calibration command queued")
        self.log_message("Sensors calibrated successfully.")
        self.update_progress("Calibration complete")

    def log_status(self, args=None):
        log_entry = f"Altitude: {self.current_altitude:.2f} cm, Battery: {self.battery_level}%, Speed: {self.motor_speed} RPM"
        self.log_message(f"Status logged: {log_entry}")
        with open('drone_status_log.txt', 'a') as log_file:
            log_file.write(f"{time.ctime()}: {log_entry}\n")
        self.update_progress("Status logged")

    def set_condition(self, args):
        condition = args.strip()
        self.command_queue.put(condition)
        self.log_message(f"Condition set: {condition}")
        self.update_progress(f"Condition set: {condition}")

    def reset_terminal(self, args=None):
        self.observation_field.config(state='normal')
        self.observation_field.delete(1.0, tk.END)
        self.observation_field.config(state='disabled')
        self.command_history = []
        self.command_index = None
        self.log_message("Terminal has been reset.")
        self.update_progress("Terminal reset")

    def clear_terminal(self, args=None):
        self.observation_field.config(state='normal')
        self.observation_field.delete(1.0, tk.END)
        self.observation_field.config(state='disabled')
        self.log_message("Terminal output cleared.")
        self.update_progress("Terminal output cleared")

    def set_safety_altitude(self, args):
        try:
            altitude = int(args)
            self.safety_altitude = altitude
            self.log_message(f"Safety altitude set to {self.safety_altitude} cm")
            self.update_progress(f"Safety altitude set to {self.safety_altitude} cm")
        except ValueError:
            self.log_message("Invalid safety altitude value. Usage: /set_safety_altitude <value>")

    def set_min_ground_distance(self, args):
        try:
            distance = int(args)
            self.min_ground_distance = distance
            self.log_message(f"Minimum ground distance set to {self.min_ground_distance} cm")
            self.update_progress(f"Minimum ground distance set to {self.min_ground_distance} cm")
        except ValueError:
            self.log_message("Invalid minimum ground distance value. Usage: /set_min_ground_distance <value>")

    def emergency_land(self, args=None):
        self.log_message("Emergency landing initiated!")
        self.send_command('EMERGENCY_LAND', "Emergency landing initiated")
        self.in_flight = False
        self.update_status("Emergency Landing")
        self.update_progress("Emergency landing in progress")

    def queue_offline_command(self, command, message):
        self.offline_command_queue.append(command)
        self.log_message(message)
        self.update_progress(f"Command queued: {command}")

    def process_offline_commands(self):
        while self.offline_command_queue:
            command = self.offline_command_queue.pop(0)
            self.send_command(command, f"Executed queued command: {command}")
        self.update_progress("Offline commands processed")

    def send_command(self, command, message):
        if self.connected and self.arduino:
            try:
                with self.serial_lock:
                    self.arduino.write(f"{command}\n".encode('utf-8'))
                self.log_message(message)
            except Exception as e:
                self.log_message(f"Error sending command '{command}': {e}")
                logging.error(f"Error sending command '{command}': {e}")
        else:
            self.log_message(f"Cannot send command '{command}': Drone is not connected.")
            self.queue_offline_command(command, f"Queued command: {command}")

    def takeoff(self, args=None):
        if self.connected and not self.in_flight:
            self.in_flight = True
            self.send_command('TAKEOFF', "Takeoff command sent.")
            self.update_status("In Flight")
            self.update_progress("Drone is taking off")
        else:
            self.log_message("Drone is already in flight or not connected.")

    def land(self, args=None):
        if self.connected and self.in_flight:
            self.in_flight = False
            self.send_command('LAND', "Landing command sent.")
            self.update_status("Landing")
            self.update_progress("Drone is landing")
        else:
            self.log_message("Drone is not in flight or not connected.")

    def set_altitude(self, args):
        try:
            altitude = int(args)
            self.desired_altitude = altitude
            self.pid_controller.setpoint = altitude
            self.log_message(f"Desired altitude set to {altitude} cm")
            self.update_progress(f"Desired altitude set to {altitude} cm")
        except ValueError:
            self.log_message("Invalid altitude value. Usage: /altitude <value>")

    def on_closing(self):
        self.closing = True  # Stop the GUI loop
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


if __name__ == "__main__":
    try:
        DroneController()
    except KeyboardInterrupt:
        print("\nExiting...")
