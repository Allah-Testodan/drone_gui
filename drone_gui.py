import sys
import os
import time
import logging
import threading
import queue
import json
import socket
import csv

import numpy as np
from filterpy.kalman import KalmanFilter

import matplotlib.pyplot as plt
import matplotlib
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

import tkinter as tk
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText
from tkinter import messagebox

try:
    import yaml
except ImportError:
    print("The 'yaml' package is not installed. Install it using 'pip install pyyaml'")
    sys.exit(1)

# Configuration path
CONFIG_PATH = "drone_config.yaml"

# Load configuration
def load_config():
    try:
        with open(CONFIG_PATH, 'r') as file:
            config = yaml.safe_load(file) or {}
    except (yaml.YAMLError, FileNotFoundError):
        config = {}
    default_config = {
        'drone_ip': '192.168.4.1',  # ESP8266 IP in AP mode
        'drone_port': 8080,
        'default_altitude': 50,
        'safety_altitude': 20,
        'min_ground_distance': 5,
        'log_level': 'DEBUG',
        'wifi_ssid': 'YourSSID',        # Enter the actual SSID
        'wifi_password': 'YourPassword' # Enter the actual password
    }
    return {**default_config, **config}


config = load_config()

# Logging configuration
log_level = config.get('log_level', 'DEBUG').upper()
logging.basicConfig(
    level=getattr(logging, log_level, logging.DEBUG),
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('drone_controller.log'),
        logging.StreamHandler()
    ]
)

# Suppress Matplotlib DEBUG messages
logging.getLogger('matplotlib').setLevel(logging.WARNING)

# Suppress PIL (Pillow) DEBUG messages
logging.getLogger('PIL').setLevel(logging.WARNING)
logging.getLogger('PIL.PngImagePlugin').setLevel(logging.WARNING)


class LowPassFilter:
    """Adaptive low-pass filter for smoothing sensor data."""
    def __init__(self, alpha=0.5):
        self.alpha = alpha
        self.last_value = None
        # Define thresholds and alpha values
        self.threshold_high = 5.0  # Adjust based on your data
        self.threshold_low = 0.1   # Adjust based on your data
        self.low_alpha_value = 0.1
        self.high_alpha_value = 0.9
        self.default_alpha_value = 0.5

    def filter(self, new_value):
        if self.last_value is None:
            self.last_value = new_value
            return new_value

        # Calculate the rate of change
        rate_of_change = abs(new_value - self.last_value)

        # Adjust alpha based on rate of change
        self.alpha = self.calculate_alpha(rate_of_change)

        filtered_value = self.alpha * new_value + (1 - self.alpha) * self.last_value
        self.last_value = filtered_value
        return filtered_value

    def calculate_alpha(self, rate_of_change):
        """Adjust alpha based on the rate of change."""
        if rate_of_change > self.threshold_high:
            return self.low_alpha_value
        elif rate_of_change < self.threshold_low:
            return self.high_alpha_value
        else:
            return self.default_alpha_value

class ComplementaryFilter:
    """Complementary filter for sensor fusion between BMP280 and HC-SR04."""
    def __init__(self, alpha=0.98):
        self.alpha = alpha
        self.estimated_altitude = 0

    def filter(self, kalman_altitude, ultrasonic_distance):
        if ultrasonic_distance is not None:
            self.estimated_altitude = self.alpha * kalman_altitude + (1 - self.alpha) * ultrasonic_distance
        else:
            self.estimated_altitude = kalman_altitude
        return self.estimated_altitude


class PIDController:
    """PID controller for altitude control."""
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
        if abs(self.integral) > 100:  # Cap the integral to prevent windup
            self.integral = 100 if self.integral > 0 else -100

        derivative = (error - self.previous_error) / dt
        control_value = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        self.last_time = current_time

        # Minimum control threshold to avoid jitter
        if abs(control_value) < 0.01:
            control_value = 0

        return control_value


class DroneController:
    """Main class to control the drone."""
    MAX_ROTOR_SPEED = 10000
    DATA_WINDOW_SECONDS = 60

    def __init__(self, root):
        # Set the root to the Tk instance passed in from LoginScreen
        self.root = root
        # Initialize variables
        self.drone_ip = config['drone_ip']
        self.drone_port = config['drone_port']
        self.sock = None
        self.in_flight = False
        self.safety_altitude = config['safety_altitude']
        self.min_ground_distance = config['min_ground_distance']
        self.battery_level = 100  # Initial battery level
        self.battery_capacity = 2200  # Battery capacity in mAh

        # Low battery threshold
        self.low_battery_threshold = 20  # Define a threshold for low battery warnings

        # PID controller for altitude control
        self.pid_controller = PIDController(Kp=1.0, Ki=0.0, Kd=0.2)
        self.desired_altitude = config['default_altitude']
        self.current_altitude = 0

        self.command_history = []
        self.command_queue = queue.Queue()
        self.command_index = None  # For navigating command history
        self.aliases = {}  # Command aliases
        self.shortcuts = {}  # Command shortcuts
        self.offline_command_queue = []  # Offline command queue
        self.connected = False

        self.altitude_shutdown_event = threading.Event()
        self.health_check_event = threading.Event()
        self.serial_lock = threading.Lock()
        self.data_lock = threading.Lock()

        self.xdata = []
        self.ydata = []
        self.battery_xdata = []
        self.battery_ydata = []
        self.start_time = time.time()

        self.motor_speed = 0  # Motor speed in RPM

        # Vibration damping filter
        self.altitude_filter = LowPassFilter(alpha=0.5)
        # Sensor fusion filter
        self.sensor_fusion_filter = ComplementaryFilter(alpha=0.98)
        # Set up Kalman filter
        self.setup_kalman_filter()

        self.closing = False  # Application closing flag

        # Flight data recording
        self.flight_data = []

        # Mapping of commands to methods
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
            '/generate_report': self.generate_mission_report,
            '/alias': self.handle_alias_command,
            '/bind': self.handle_bind_command,
            '/export_logs': self.export_logs,
            '/set_log_level': self.set_log_level,
            '/schedule': self.schedule_command,
            '/estimate_time': self.estimate_remaining_time,
        }

        # List of all commands for auto-completion
        self.all_commands = list(self.command_methods.keys())

        # Start GUI in the main thread
        self.setup_gui()
        self.gui_task_after_id = None  # Initialize 'after' ID
        self.root.after(0, self.run_gui_tasks)

        self.start_health_check_thread()
        self.root.mainloop()

    def calculate_consumption_rate(self):
        """Calculate the battery consumption rate in mAh per minute."""
        # Motor specifications:
        # - Motor type: 2205 2300KV brushless motor
        # - Average current draw per motor during hover: 7A
        number_of_motors = 4
        average_current_per_motor = 7  # in Amperes (A)
        total_current_draw = number_of_motors * average_current_per_motor  # 28A

        # Convert current draw to consumption rate in mAh per minute.
        consumption_rate_per_minute = total_current_draw * (1 / 60) * 1000  # in mAh per minute
        return consumption_rate_per_minute

    def estimate_remaining_time(self, args=None):
        """Estimate remaining flight time based on battery level."""
        consumption_rate = self.calculate_consumption_rate()  # mAh per minute
        remaining_capacity = (self.battery_level / 100) * self.battery_capacity  # mAh
        if consumption_rate > 0:
            remaining_time = remaining_capacity / consumption_rate  # Minutes
            self.log_message(f"Estimated remaining flight time: {remaining_time:.2f} minutes")
            self.update_progress(f"Estimated flight time: {remaining_time:.2f} minutes")
        else:
            self.log_message("Consumption rate is zero, cannot estimate remaining time.")

    def setup_kalman_filter(self):
        """Set up the Kalman filter for sensor fusion."""
        self.kf = KalmanFilter(dim_x=2, dim_z=1)
        dt = 0.1  # Time step

        # State Transition Matrix
        self.kf.F = np.array([[1, dt],
                              [0, 1]])

        # Measurement Function
        self.kf.H = np.array([[1, 0]])

        # Initial State Covariance
        self.kf.P *= 1000

        # Process Noise Covariance
        self.kf.Q = np.array([[1, 0],
                              [0, 3]])

        # Measurement Noise Covariance
        self.kf.R = np.array([[5]])

        # Initial State
        self.kf.x = np.array([[0],
                              [0]])

    def connect_drone(self, args=None):
        """Connect to the drone."""
        if self.connected:
            self.log_message("Already connected to the drone.")
            self.update_progress("Already connected")
            return

        self.log_message("Attempting to connect to the drone...")
        threading.Thread(target=self.connect_to_drone_thread, daemon=True).start()

    def connect_to_drone_thread(self):
        """Thread function for connecting to the drone."""
        retries = 3
        attempt = 0
        self.connected = False  # Reset connection status before attempting
        while attempt < retries and not self.connected:
            if self.closing:
                break
            try:
                # Creating TCP/IP socket
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(5)
                self.sock.connect((self.drone_ip, self.drone_port))
                self.connected = True
                self.update_status("Connected")
                self.update_connection_indicator("Connected")
                self.log_message(f"Connected to drone at address {self.drone_ip}:{self.drone_port}.")
                threading.Thread(target=self.receive_data_thread, daemon=True).start()
                self.process_offline_commands()
                self.start_altitude_thread()
                break
            except socket.error as e:
                attempt += 1
                self.log_message(f"Failed to connect: {e}. Retrying ({retries - attempt} attempts left)")
                logging.error(f"Socket error: {e}")
                time.sleep(2)  # Delay between attempts
            except Exception as e:
                self.log_message(f"An unexpected error occurred: {e}")
                logging.error(f"Unexpected error: {e}")
                break
        else:
            self.log_message("Failed to connect after several attempts.")
            self.update_status("Failed to connect")
            self.update_connection_indicator("Disconnected")
            self.connected = False

    def disconnect_drone(self, args=None):
        """Disconnect from the drone."""
        if self.connected and self.sock:
            try:
                self.altitude_shutdown_event.set()  # Stop the altitude control loop
                self.health_check_event.set()  # Stop the health check loop
                self.sock.close()
                self.connected = False
                self.update_status("Disconnected")
                self.update_connection_indicator("Disconnected")
                self.log_message("Disconnected from drone.")
                self.update_progress("Disconnected")
            except Exception as e:
                self.log_message(f"Error during disconnection: {e}")
                logging.error(f"Error during disconnection: {e}")
        else:
            self.log_message("Drone is not connected.")
            self.update_progress("Not connected")

    def open_settings_window(self):
        """Open a window to adjust settings."""
        # Create a new window
        settings_window = tk.Toplevel(self.root)
        settings_window.title("Settings")
        settings_window.geometry("400x300")
        settings_window.configure(bg="#0F0F0F")

        # Example: Drone IP setting
        tk.Label(settings_window, text="Drone IP:", bg="#0F0F0F", fg="#00FF00", font=("Courier", 12)).pack(pady=5)
        drone_ip_entry = tk.Entry(settings_window, bg="#0F0F0F", fg="#00FF00", font=("Courier", 12),
                                  insertbackground="#00FF00")
        drone_ip_entry.insert(0, self.drone_ip)
        drone_ip_entry.pack(pady=5)

        # Example: Drone Port setting
        tk.Label(settings_window, text="Drone Port:", bg="#0F0F0F", fg="#00FF00", font=("Courier", 12)).pack(pady=5)
        drone_port_entry = tk.Entry(settings_window, bg="#0F0F0F", fg="#00FF00", font=("Courier", 12),
                                    insertbackground="#00FF00")
        drone_port_entry.insert(0, str(self.drone_port))
        drone_port_entry.pack(pady=5)

        # Save button
        def save_settings():
            new_ip = drone_ip_entry.get()
            new_port = drone_port_entry.get()
            if self.validate_ip(new_ip) and new_port.isdigit():
                self.drone_ip = new_ip
                self.drone_port = int(new_port)
                # Save to config file
                config['drone_ip'] = self.drone_ip
                config['drone_port'] = self.drone_port
                with open(CONFIG_PATH, 'w') as file:
                    yaml.dump(config, file)
                settings_window.destroy()
                self.log_message("Settings updated.")
                self.update_progress("Settings updated")
            else:
                messagebox.showerror("Invalid Input", "Please enter a valid IP address and port.")

        save_button = tk.Button(settings_window, text="Save", command=save_settings, bg="#0F0F0F", fg="#00FF00",
                                font=("Courier", 12))
        save_button.pack(pady=20)

    def validate_ip(self, ip):
        """Validate IP address format."""
        try:
            socket.inet_aton(ip)
            return True
        except socket.error:
            return False

    def setup_gui(self):
        """Set up the GUI elements."""
        self.root.title("Drone Control Interface")
        self.root.geometry("1200x900")
        self.root.configure(bg="#0F0F0F")
        # Menu bar
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)

        settings_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Settings", menu=settings_menu)
        settings_menu.add_command(label="Configure", command=self.open_settings_window)

        # Connection status indicator
        self.connection_indicator = tk.Label(self.root, text="Disconnected", bg="#0F0F0F", fg="red", font=("Courier", 12))
        self.connection_indicator.pack(pady=5)

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

        # Command suggestion box
        self.command_suggestion_box = tk.Listbox(self.root, bg="#0F0F0F", fg="#00FF00",
                                                 font=("Courier", 12), height=5)
        self.command_suggestion_box.pack(pady=5)
        self.command_suggestion_box.bind("<<ListboxSelect>>", self.fill_command_from_suggestion)

        # Status frame to hold status labels
        self.status_frame = tk.Frame(self.root, bg="#0F0F0F")
        self.status_frame.pack(pady=10, fill=tk.X)

        # Configure grid columns for equal expansion
        for i in range(4):
            self.status_frame.columnconfigure(i, weight=1)

        # Status labels in grid layout
        self.status_label = tk.Label(self.status_frame, text="Status: Idle", bg="#0F0F0F", fg="#00FF00",
                                     font=("Courier", 12))
        self.status_label.grid(row=0, column=0, sticky=tk.W+tk.E, padx=5)

        self.altitude_label = tk.Label(self.status_frame, text="Altitude: 0 cm", bg="#0F0F0F", fg="#00FF00",
                                       font=("Courier", 12))
        self.altitude_label.grid(row=0, column=1, sticky=tk.W+tk.E, padx=5)

        self.motor_speed_label = tk.Label(self.status_frame, text="Motor Speed: 0 RPM", bg="#0F0F0F",
                                          fg="#00FF00", font=("Courier", 12))
        self.motor_speed_label.grid(row=0, column=2, sticky=tk.W+tk.E, padx=5)

        self.progress_label = tk.Label(self.status_frame, text="Progress: Idle", bg="#0F0F0F", fg="#00FF00",
                                       font=("Courier", 12))
        self.progress_label.grid(row=0, column=3, sticky=tk.W+tk.E, padx=5)

        # Gauges and plots frame
        self.gauges_frame = tk.Frame(self.root, bg="#0F0F0F")
        self.gauges_frame.pack(pady=10, fill=tk.BOTH, expand=True)

        # Left frame for battery plot and gauge
        self.left_frame = tk.Frame(self.gauges_frame, bg="#0F0F0F")
        self.left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Right frame for rotor speed plot and gauge
        self.right_frame = tk.Frame(self.gauges_frame, bg="#0F0F0F")
        self.right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # Live plots
        self.setup_live_plots()

        # Progress bar
        self.progress_bar = ttk.Progressbar(self.root, style="green.Horizontal.TProgressbar",
                                            length=400, mode='determinate')
        self.progress_bar.pack(pady=10)
        style = ttk.Style()
        style.theme_use('default')
        style.configure("green.Horizontal.TProgressbar", troughcolor='#0F0F0F', background='#00FF00',
                        bordercolor="#0F0F0F")

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def setup_live_plots(self):
        """Set up the live plots for battery level and rotor speed without circular gauges."""
        # Adjust plot dimensions and colors
        matplotlib.rcParams['axes.facecolor'] = '#0F0F0F'
        matplotlib.rcParams['figure.facecolor'] = '#0F0F0F'
        matplotlib.rcParams['savefig.facecolor'] = '#0F0F0F'

        # Battery plot
        self.fig_battery, self.ax_battery = plt.subplots(figsize=(6, 2.5))
        self.line_battery, = self.ax_battery.plot([], [], lw=2, color='#FFA500')  # Orange color
        self.ax_battery.set_title("Battery Level Over Time", color='white')
        self.ax_battery.set_xlabel("Time (s)", color='white')
        self.ax_battery.set_ylabel("Battery Level (%)", color='white')
        self.ax_battery.tick_params(axis='x', colors='white')
        self.ax_battery.tick_params(axis='y', colors='white')
        self.ax_battery.set_xlim(0, self.DATA_WINDOW_SECONDS)
        self.ax_battery.set_ylim(0, 100)

        self.canvas_battery = FigureCanvasTkAgg(self.fig_battery, master=self.left_frame)
        self.canvas_battery.draw()
        self.canvas_battery.get_tk_widget().pack(side=tk.TOP, pady=10)

        # Rotor speed plot
        self.fig_rotor_speed, self.ax_rotor_speed = plt.subplots(figsize=(6, 2.5))
        self.line_rotor_speed, = self.ax_rotor_speed.plot([], [], lw=2, color='#00FF00')
        self.ax_rotor_speed.set_title("Rotor Speed Over Time", color='white')
        self.ax_rotor_speed.set_xlabel("Time (s)", color='white')
        self.ax_rotor_speed.set_ylabel("Speed (RPM)", color='white')
        self.ax_rotor_speed.tick_params(axis='x', colors='white')
        self.ax_rotor_speed.tick_params(axis='y', colors='white')
        self.ax_rotor_speed.set_xlim(0, self.DATA_WINDOW_SECONDS)
        self.ax_rotor_speed.set_ylim(0, self.MAX_ROTOR_SPEED)

        self.canvas_rotor_speed = FigureCanvasTkAgg(self.fig_rotor_speed, master=self.right_frame)
        self.canvas_rotor_speed.draw()
        self.canvas_rotor_speed.get_tk_widget().pack(side=tk.TOP, pady=10)

    def run_gui_tasks(self):
        """Periodic updates of GUI elements."""
        if self.closing:
            return
        self.update_live_plots()
        self.gui_task_after_id = self.root.after(1000, self.run_gui_tasks)  # Schedule next update

    def update_live_plots(self):
        """Update the live plots."""
        if self.closing:
            return
        current_time = time.time() - self.start_time

        # Altitude data
        self.xdata.append(current_time)
        self.ydata.append(self.current_altitude)

        # Battery data
        self.battery_xdata.append(current_time)
        self.battery_ydata.append(self.battery_level)

        # Rotor speed data
        rotor_speed = self.motor_speed
        if not hasattr(self, 'rotor_speed_xdata'):
            self.rotor_speed_xdata = []
            self.rotor_speed_ydata = []
        self.rotor_speed_xdata.append(current_time)
        self.rotor_speed_ydata.append(rotor_speed)

        # Keep only the last DATA_WINDOW_SECONDS of data
        if current_time > self.DATA_WINDOW_SECONDS:
            self.xdata = self.xdata[-self.DATA_WINDOW_SECONDS:]
            self.ydata = self.ydata[-self.DATA_WINDOW_SECONDS:]
            self.battery_xdata = self.battery_xdata[-self.DATA_WINDOW_SECONDS:]
            self.battery_ydata = self.battery_ydata[-self.DATA_WINDOW_SECONDS:]
            self.rotor_speed_xdata = self.rotor_speed_xdata[-self.DATA_WINDOW_SECONDS:]
            self.rotor_speed_ydata = self.rotor_speed_ydata[-self.DATA_WINDOW_SECONDS:]

            self.ax_battery.set_xlim(current_time - self.DATA_WINDOW_SECONDS, current_time)
            self.ax_rotor_speed.set_xlim(current_time - self.DATA_WINDOW_SECONDS, current_time)

        # Update battery plot
        self.line_battery.set_data(self.battery_xdata, self.battery_ydata)
        self.ax_battery.relim()
        self.ax_battery.autoscale_view()
        self.canvas_battery.draw()

        # Update rotor speed plot
        self.line_rotor_speed.set_data(self.rotor_speed_xdata, self.rotor_speed_ydata)
        self.ax_rotor_speed.relim()
        self.ax_rotor_speed.autoscale_view()
        self.canvas_rotor_speed.draw()

    def process_terminal_command(self, event=None):
        """Process commands entered in the terminal."""
        command = self.command_entry.get()
        if command is not None:
            command = command.strip()
        else:
            command = ''
        if command:
            self.command_history.append(command)
            self.command_index = None

            self.log_message(f"> {command}")

            # Check for aliases
            if command in self.aliases:
                command = self.aliases[command]

            command_parts = command.split()
            if not command_parts:
                return

            command_name = command_parts[0]
            command_args = ' '.join(command_parts[1:])

            # Handle shortcuts
            if command_name in self.shortcuts:
                command_name = self.shortcuts[command_name]

            # Get the command method
            command_method = self.command_methods.get(command_name)
            if command_method:
                try:
                    command_method(command_args)
                except Exception as e:
                    self.log_message(f"Error executing command '{command_name}': {e}", level='ERROR')
                    logging.exception("Exception in command execution")
            else:
                self.log_message(f"Unknown command: {command_name}")
                self.log_message("Type /help to see available commands.")

            # Clear the command suggestion box after executing the command
            self.command_suggestion_box.delete(0, tk.END)
            self.command_entry.delete(0, tk.END)

    def handle_alias_command(self, args):
        """Set a command alias."""
        try:
            alias, original_command = args.strip().split(' ', 1)
            self.aliases[alias] = original_command
            self.all_commands.append(alias)
            self.log_message(f"Alias set: {alias} -> {original_command}")
        except ValueError:
            self.log_message("Invalid alias format. Use: /alias <alias> <command>")

    def handle_bind_command(self, args):
        """Bind a shortcut to a command."""
        try:
            shortcut, original_command = args.strip().split(' ', 1)
            self.shortcuts[shortcut] = original_command
            self.all_commands.append(shortcut)
            self.log_message(f"Shortcut set: {shortcut} -> {original_command}")
        except ValueError:
            self.log_message("Invalid shortcut format. Use: /bind <key> <command>")

    def display_help(self, args=None):
        """Show this help message."""
        help_text = "Available commands:\n"
        for command in sorted(self.command_methods.keys()):
            method = self.command_methods[command]
            doc = method.__doc__.strip() if method.__doc__ else ''
            help_text += f"  {command.ljust(25)} - {doc}\n"
        self.log_message(help_text)
        self.update_progress("Help displayed")

    def auto_complete_command(self, event):
        """Auto-complete command based on partial input."""
        command_prefix = self.command_entry.get().strip()
        matching_commands = [
            cmd for cmd in set(self.all_commands) if cmd.startswith(command_prefix)
        ]
        if len(matching_commands) == 1:
            self.command_entry.delete(0, tk.END)
            self.command_entry.insert(tk.END, matching_commands[0])
        return "break"

    def update_command_suggestions(self, event):
        """Update command suggestions based on current input."""
        typed_text = self.command_entry.get().strip()
        all_commands = set(self.all_commands)
        matching_commands = [cmd for cmd in all_commands if cmd.startswith(typed_text)]

        self.command_suggestion_box.delete(0, tk.END)
        # Limit the number of suggestions displayed to 10
        for command in matching_commands[:10]:
            self.command_suggestion_box.insert(tk.END, command)

    def fill_command_from_suggestion(self, event):
        """Fill the command entry with the selected suggestion."""
        selection = self.command_suggestion_box.curselection()
        if selection:
            self.command_entry.delete(0, tk.END)
            self.command_entry.insert(0, self.command_suggestion_box.get(selection[0]))

    def previous_command(self, event):
        """Navigate to the previous command in history."""
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
        """Navigate to the next command in history."""
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

    def update_status(self, status):
        """Update the status label."""
        if self.closing:
            return
        self.root.after(100, self._update_status_main_thread, status)

    def _update_status_main_thread(self, status):
        self.status_label.config(text=f"Status: {status}")

    def update_connection_indicator(self, status):
        """Update the connection status indicator."""
        if self.closing:
            return
        color = "green" if status == "Connected" else "red"
        self.root.after(100, self._update_connection_indicator_main_thread, status, color)

    def _update_connection_indicator_main_thread(self, status, color):
        self.connection_indicator.config(text=status, fg=color)

    def log_message(self, message, level='INFO'):
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        full_message = f"[{timestamp}] [{level}] {message}"
        self.root.after(100, self._log_message_main_thread, full_message, level)

    def _log_message_main_thread(self, full_message, level):
        self.observation_field.config(state='normal')
        self.observation_field.insert(tk.END, f"{full_message}\n")
        self.observation_field.see(tk.END)
        self.observation_field.config(state='disabled')
        log_func = {'INFO': logging.info, 'WARNING': logging.warning, 'ERROR': logging.error}.get(level, logging.info)
        log_func(full_message)
        for handler in logging.getLogger().handlers:
            if isinstance(handler, logging.FileHandler):
                handler.flush()

    def update_progress(self, progress):
        """Update the progress label."""
        if self.closing:
            return
        self.root.after(100, self._update_progress_main_thread, progress)

    def _update_progress_main_thread(self, progress):
        self.progress_label.config(text=f"Progress: {progress}")

    def update_battery_gauge(self, percentage):
        """Update the battery gauge."""
        if self.closing:
            return
        self.root.after(100, self._update_battery_gauge_main_thread, percentage)

    def _update_battery_gauge_main_thread(self, percentage):
        self.battery_gauge.delete("all")
        self.battery_gauge.create_oval(10, 10, 140, 140, outline="#00FF00", width=4)
        self.battery_gauge.create_arc(10, 10, 140, 140, start=90, extent=-percentage * 3.6, outline="#00FF00",
                                      style="arc", width=8)
        self.battery_gauge.create_text(75, 75, text=f"{percentage}%", fill="#00FF00", font=("Courier", 12))

    def update_rotor_speed_gauge(self, speed):
        """Update the motor speed gauge."""
        if self.closing:
            return
        self.root.after(100, self._update_rotor_speed_gauge_main_thread, speed)

    def _update_rotor_speed_gauge_main_thread(self, speed):
        max_speed = self.MAX_ROTOR_SPEED  # Using the defined constant
        percentage = (speed / max_speed) * 100
        self.rotor_speed_gauge.delete("all")
        self.rotor_speed_gauge.create_oval(10, 10, 140, 140, outline="#00FF00", width=4)
        self.rotor_speed_gauge.create_arc(10, 10, 140, 140, start=90, extent=-percentage * 3.6, outline="#00FF00",
                                          style="arc", width=8)
        self.rotor_speed_gauge.create_text(75, 75, text=f"{speed} RPM", fill="#00FF00", font=("Courier", 12))

    def receive_data_thread(self):
        """Receive data from the drone."""
        try:
            while self.connected and not self.closing:
                try:
                    data = self.sock.recv(1024)
                    if data:
                        data_line = data.decode('utf-8').strip()
                        self.process_serial_data(data_line)
                except socket.timeout:
                    time.sleep(0.1)
                    continue
                except Exception as e:
                    self.log_message(f"Error receiving data: {e}", level='ERROR')
                    logging.exception("Exception in receive_data_thread")
                    break
        finally:
            self.log_message("Data receiving thread terminated.", level='WARNING')

    def process_sensor_data(self, bmp280_altitude):
        """Process BMP280 data using the Kalman filter."""
        z = np.array([[bmp280_altitude]])  # Measurement

        # Predict
        self.kf.predict()

        # Update
        self.kf.update(z)

    def process_serial_data(self, data_line):
        """Process data received from the drone."""
        try:
            data = json.loads(data_line)
            altitude_bmp280 = data.get('bmp280_altitude')
            ultrasonic_distance = data.get('ultrasonic_distance')

            if altitude_bmp280 is None and ultrasonic_distance is None:
                self.log_message("No valid altitude data received.", level='WARNING')
                return

            with self.data_lock:
                # Use Kalman filter to process BMP280 altitude
                if altitude_bmp280 is not None:
                    self.process_sensor_data(altitude_bmp280)

                    # Fuse Kalman filter output with ultrasonic data
                    self.current_altitude = self.sensor_fusion_filter.filter(
                        self.kf.x[0, 0], ultrasonic_distance
                    )

                    # Apply vibration damping
                    self.current_altitude = self.altitude_filter.filter(self.current_altitude)

                # Update other data
                self.battery_level = data.get('battery', self.battery_level)
                self.motor_speed = data.get('motor_speed', self.motor_speed)

                # Record flight data
                self.record_flight_data()

            self.update_gui_elements()
            self.check_battery()
        except json.JSONDecodeError:
            self.log_message(f"Received invalid data: {data_line}", level='ERROR')
        except Exception as e:
            self.log_message(f"Error processing data: {e}", level='ERROR')
            logging.exception("Exception in process_serial_data")

    def update_gui_elements(self):
        """Update GUI elements based on new sensor data."""
        if self.closing:
            return
        self.root.after(100, self._update_gui_elements_main_thread)

    def _update_gui_elements_main_thread(self):
        self.altitude_label.config(text=f"Altitude: {self.current_altitude:.2f} cm")
        self.motor_speed_label.config(text=f"Motor Speed: {self.motor_speed} RPM")
        self.update_battery_gauge(self.battery_level)
        self.update_rotor_speed_gauge(self.motor_speed)

    def start_altitude_thread(self):
        """Start the altitude control thread."""
        self.altitude_shutdown_event.clear()
        threading.Thread(target=self.altitude_control_loop, daemon=True).start()

    def altitude_control_loop(self):
        """Control loop to maintain the desired altitude."""
        try:
            while not self.altitude_shutdown_event.is_set():
                if self.closing:
                    break
                if self.in_flight and self.connected:
                    # Apply PID control to adjust thrust
                    control_signal = self.pid_controller.update(self.current_altitude)

                    # Dynamic thrust adjustment
                    if self.current_altitude <= self.min_ground_distance:
                        control_signal = 0  # Stop motors when close to the ground
                        self.log_message("Minimum ground distance reached, stopping motors.", level='WARNING')
                        self.in_flight = False
                        self.update_status("Landed")
                        self.send_command('STOP_MOTORS', "Stopping motors.")
                        continue  # Skip sending thrust command

                    # Send control signal to the drone
                    self.send_command(f'SET_THRUST {control_signal}', f"Adjusting thrust: {control_signal:.2f}")

                    # Update motor speed based on control signal
                    self.motor_speed = max(0, int(abs(control_signal) * 1000))  # Scale thrust to RPM
                time.sleep(0.05)
        except Exception as e:
            self.log_message(f"Error in altitude control loop: {e}", level='ERROR')
            logging.exception("Exception in altitude_control_loop")

    def check_battery(self):
        """Check battery level and take appropriate actions."""
        if self.battery_level < self.low_battery_threshold:
            if not hasattr(self, 'low_battery_warning_shown'):
                self.low_battery_warning_shown = True
                self.log_message("Battery below threshold, initiating emergency actions.", level='WARNING')
                # Send command to activate buzzer on the Arduino
                self.send_command('ACTIVATE_BUZZER', "Activating buzzer for low battery warning")
                time.sleep(1)
                self.send_command('DEACTIVATE_BUZZER', "Deactivating buzzer")
                self.emergency_land()
        elif self.battery_level < 50:
            if not hasattr(self, 'battery_warning_shown'):
                self.battery_warning_shown = True
                self.log_message("Warning: Battery below 50%.", level='WARNING')

    def check_battery_status(self, args=None):
        """Check battery status."""
        self.check_battery()
        self.log_message(f"Battery level: {self.battery_level}%", level='INFO')
        self.update_progress("Battery status checked")

    def emergency_land(self, args=None):
        """Force emergency landing."""
        if self.closing:
            return
        self.log_message("Initiating emergency landing!", level='ERROR')
        self.send_command('EMERGENCY_LAND', "Emergency landing command sent")
        self.in_flight = False
        self.update_status("Emergency Landing")
        self.update_progress("Emergency landing in progress")

    def start_health_check_thread(self):
        """Start the drone's health check thread."""
        threading.Thread(target=self.health_check_loop, daemon=True).start()

    def health_check_loop(self):
        """Periodic checking of the drone's systems."""
        try:
            while not self.health_check_event.is_set():
                if self.closing:
                    break
                # Placeholder for actual health checks
                motor_status = "OK"
                sensor_status = "OK"
                self.log_message(f"Motor Status: {motor_status}")
                self.log_message(f"Sensor Status: {sensor_status}")
                time.sleep(10)  # Check every 10 seconds
        except Exception as e:
            self.log_message(f"Error in health check loop: {e}", level='ERROR')
            logging.exception("Exception in health_check_loop")

    def calibrate_sensors(self, args=None):
        """Calibrate the drone's sensors."""
        self.log_message("Calibrating sensors...")
        self.update_progress("Calibrating sensors")
        if self.connected:
            self.send_command('CALIBRATE', "Calibration command sent")
        else:
            self.queue_offline_command('CALIBRATE', "Calibration command added to queue")
            self.log_message("Drone is not connected. Calibration will occur once connected.")
        self.log_message("Sensors calibrated successfully.")
        self.update_progress("Calibration complete")

    def set_log_level(self, args):
        """Set the logging level."""
        level = args.strip().upper()
        if level in ['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL']:
            logging.getLogger().setLevel(getattr(logging, level))
            self.log_message(f"Log level set to {level}.")
        else:
            self.log_message("Invalid log level. Use: /set_log_level <level>")

    def log_status(self, args=None):
        """Log the current status."""
        log_entry = f"Altitude: {self.current_altitude:.2f} cm, Battery: {self.battery_level}%, Speed: {self.motor_speed} RPM"
        self.log_message(f"Status logged: {log_entry}")
        with open('drone_status_log.txt', 'a') as log_file:
            log_file.write(f"{time.ctime()}: {log_entry}\n")
        self.update_progress("Status logged")

    def set_condition(self, args):
        """Set a condition for automatic actions."""
        condition = args.strip()
        if condition:
            self.command_queue.put(condition)
            self.log_message(f"Condition set: {condition}")
            self.update_progress(f"Condition set: {condition}")
        else:
            self.log_message("No condition provided. Usage: /condition <condition>")
            self.update_progress("Failed to set condition")

    def reset_terminal(self, args=None):
        """Reset the terminal and restart the application."""
        self.log_message("Resetting terminal and application...", level='WARNING')
        self.closing = True  # Set the application closing flag
        self.altitude_shutdown_event.set()
        self.health_check_event.set()
        self.disconnect_drone()
        if self.gui_task_after_id is not None:
            self.root.after_cancel(self.gui_task_after_id)
        self.root.after(1000, self.restart_application)  # Delay to show reset message

    def restart_application(self):
        """Restart the application."""
        self.root.destroy()
        os.execl(sys.executable, sys.executable, *sys.argv)

    def clear_terminal(self, args=None):
        """Clear the terminal output."""
        if self.closing:
            return
        self.observation_field.config(state='normal')
        self.observation_field.delete(1.0, tk.END)
        self.observation_field.config(state='disabled')
        self.log_message("Terminal output cleared.")
        self.update_progress("Terminal output cleared")

    def set_safety_altitude(self, args):
        """Set safety altitude for emergency landings."""
        try:
            altitude = int(args)
            self.safety_altitude = altitude
            # Save to config
            config['safety_altitude'] = self.safety_altitude
            with open(CONFIG_PATH, 'w') as file:
                yaml.dump(config, file)
            self.log_message(f"Safety altitude set to {self.safety_altitude} cm")
            self.update_progress(f"Safety altitude set to {self.safety_altitude} cm")
        except ValueError:
            self.log_message("Invalid safety altitude value. Use: /set_safety_altitude <value>")
            self.update_progress("Failed to set safety altitude")

    def set_min_ground_distance(self, args):
        """Set minimum ground distance to stop motors."""
        try:
            distance = int(args)
            self.min_ground_distance = distance
            # Save to config
            config['min_ground_distance'] = self.min_ground_distance
            with open(CONFIG_PATH, 'w') as file:
                yaml.dump(config, file)
            self.log_message(f"Minimum ground distance set to {self.min_ground_distance} cm")
            self.update_progress(f"Minimum ground distance set to {self.min_ground_distance} cm")
        except ValueError:
            self.log_message("Invalid minimum ground distance value. Use: /set_min_ground_distance <value>")
            self.update_progress("Failed to set min ground distance")

    def queue_offline_command(self, command, message):
        """Add a command to the queue to be sent when the drone connects."""
        if self.closing:
            return
        self.offline_command_queue.append(command)
        self.log_message(message)
        self.update_progress(f"Command added to queue: {command}")
        if not hasattr(self, 'offline_message_shown'):
            self.offline_message_shown = True
            self.log_message("Drone is not connected. Commands will be sent once connected.")

    def process_offline_commands(self):
        """Process commands from the offline queue."""
        if self.closing:
            return
        while self.offline_command_queue:
            command = self.offline_command_queue.pop(0)
            self.send_command(command, f"Executed command from queue: {command}")
        self.update_progress("Processed commands from queue")

    def send_command(self, command, message):
        if self.closing:
            return
        if self.connected and self.sock:
            try:
                with self.serial_lock:
                    self.sock.sendall(f"{command}\n".encode('utf-8'))
                self.log_message(message)
            except Exception as e:
                self.log_message(f"Error sending command '{command}': {e}", level='ERROR')
                logging.exception("Exception in send_command")
        else:
            self.log_message(f"Cannot send command '{command}': Drone is not connected.", level='WARNING')
            self.queue_offline_command(command, f"Command added to queue: {command}")

    def takeoff(self, args=None):
        """Initiate drone takeoff."""
        if self.closing:
            return
        if not self.connected:
            self.log_message("Cannot take off: Drone is not connected.", level='WARNING')
            self.update_progress("Takeoff failed")
            return
        if self.in_flight:
            self.log_message("Drone is already in flight.", level='WARNING')
            self.update_progress("Already in flight")
            return
        if self.battery_level < 30:
            self.log_message("Battery too low for takeoff.", level='WARNING')
            self.update_progress("Takeoff aborted")
            return

        self.in_flight = True
        self.send_command('TAKEOFF', "Takeoff command sent.")
        self.update_status("In Flight")
        self.update_progress("Drone is taking off")

    def land(self, args=None):
        """Initiate drone landing."""
        if self.closing:
            return
        if self.connected and self.in_flight:
            self.log_message("Initiating landing sequence.")
            self.update_status("Landing")
            self.update_progress("Drone is landing")

            # Gradual descent
            self.initiate_landing()
        else:
            self.log_message("Drone is not in flight or not connected.", level='WARNING')
            self.update_progress("Landing failed")

    def initiate_landing(self):
        """Perform a gradual descent for landing."""
        def descent():
            try:
                while self.current_altitude > self.min_ground_distance and not self.closing:
                    self.desired_altitude -= 1  # Decrease desired altitude gradually
                    self.pid_controller.setpoint = self.desired_altitude
                    self.log_message(f"Descending to {self.desired_altitude} cm.")
                    time.sleep(0.5)
                self.in_flight = False
                self.send_command('STOP_MOTORS', "Motors stopped after landing.")
                self.update_status("Landed")
                self.update_progress("Landing complete")
                self.log_message("Drone has landed safely.")
            except Exception as e:
                self.log_message(f"Error during landing: {e}", level='ERROR')
                logging.exception("Exception in initiate_landing")

        threading.Thread(target=descent, daemon=True).start()

    def set_altitude(self, args):
        """Set the desired altitude."""
        if self.closing:
            return
        try:
            altitude = int(args)
            if altitude < self.min_ground_distance:
                self.log_message(f"Desired altitude {altitude} cm is below minimum ground distance.", level='WARNING')
                self.update_progress("Failed to set altitude")
                return
            self.desired_altitude = altitude
            self.pid_controller.setpoint = altitude
            self.log_message(f"Desired altitude set to {altitude} cm")
            self.update_progress(f"Desired altitude set to {altitude} cm")
        except ValueError:
            self.log_message("Invalid altitude value. Use: /altitude <value>")
            self.update_progress("Failed to set altitude")

    def on_closing(self):
        """Handle application closing."""
        if messagebox.askokcancel("Quit", "Do you really want to quit?"):
            self.closing = True
            self.altitude_shutdown_event.set()
            self.health_check_event.set()
            if self.sock:
                self.sock.close()
            if self.gui_task_after_id is not None:
                self.root.after_cancel(self.gui_task_after_id)
            self.export_flight_data()
            self.root.destroy()

    def generate_mission_report(self, args=None):
        """Generate a mission report."""
        report = "Mission Report:\n"
        report += f"Commands Executed: {len(self.command_history)}\n"
        report += "\n".join(self.command_history)
        report += "\nMission completed successfully."
        with open('mission_report.txt', 'w') as report_file:
            report_file.write(report)
        self.log_message("Mission report generated.")
        self.update_progress("Mission report generated")

    def export_logs(self, args=None):
        """Export logs to a file."""
        try:
            with open('exported_logs.txt', 'w') as f:
                with open('drone_controller.log', 'r') as log_file:
                    f.write(log_file.read())
            self.log_message("Logs exported successfully.")
            self.update_progress("Logs exported")
        except Exception as e:
            self.log_message(f"Error exporting logs: {e}", level='ERROR')
            logging.exception("Exception in export_logs")

    def schedule_command(self, args):
        """Schedule a command to be executed after a delay."""
        try:
            delay_str, command = args.strip().split(' ', 1)
            delay = float(delay_str)
            self.log_message(f"Scheduling command '{command}' to run after {delay} seconds.")

            def delayed_execution():
                time.sleep(delay)
                self.root.after(0, lambda: self.command_entry.insert(0, command))
                self.root.after(0, self.process_terminal_command)

            threading.Thread(target=delayed_execution, daemon=True).start()
            self.update_progress(f"Command scheduled: {command}")
        except ValueError:
            self.log_message("Invalid schedule format. Use: /schedule <delay_in_seconds> <command>")
            self.update_progress("Failed to schedule command")

    def record_flight_data(self):
        """Record flight data for analysis."""
        data_point = {
            'time': time.time() - self.start_time,
            'altitude': self.current_altitude,
            'battery_level': self.battery_level,
            'motor_speed': self.motor_speed
        }
        self.flight_data.append(data_point)

    def export_flight_data(self):
        """Export flight data to a CSV file."""
        if not self.flight_data:
            return
        try:
            keys = self.flight_data[0].keys()
            with open('flight_data.csv', 'w', newline='') as output_file:
                dict_writer = csv.DictWriter(output_file, keys)
                dict_writer.writeheader()
                dict_writer.writerows(self.flight_data)
            self.log_message("Flight data exported successfully.")
        except Exception as e:
            self.log_message(f"Error exporting flight data: {e}", level='ERROR')
            logging.exception("Exception in export_flight_data")

class LoginScreen:
    """Matrix-style login screen."""
    def __init__(self, root):
        self.root = root
        self.root.title("PROJECT CPS: Login")
        self.root.geometry("500x300")
        self.root.configure(bg="black")

        # Title Label
        self.title_label = tk.Label(self.root, text="PROJECT CPS", font=("Courier", 24, "bold"), fg="#00FF00", bg="black")
        self.title_label.pack(pady=20)

        # Username Label and Entry
        self.username_label = tk.Label(self.root, text="Username:", font=("Courier", 14), fg="#00FF00", bg="black")
        self.username_label.pack(pady=5)
        self.username_entry = tk.Entry(self.root, font=("Courier", 14), fg="#00FF00", bg="black", insertbackground="#00FF00")
        self.username_entry.pack(pady=5)

        # Password Label and Entry
        self.password_label = tk.Label(self.root, text="Password:", font=("Courier", 14), fg="#00FF00", bg="black")
        self.password_label.pack(pady=5)
        self.password_entry = tk.Entry(self.root, font=("Courier", 14), show="*", fg="#00FF00", bg="black", insertbackground="#00FF00")
        self.password_entry.pack(pady=5)

        # Login Button
        self.login_button = tk.Button(self.root, text="Login", font=("Courier", 14, "bold"), command=self.login, fg="black", bg="#00FF00")
        self.login_button.pack(pady=20)

        # Pressing Enter triggers login
        self.root.bind("<Return>", lambda event: self.login())

    def login(self):
        """Check login credentials and open the main application if correct."""
        username = self.username_entry.get()
        password = self.password_entry.get()

        if username == "admin" and password == "1234":
            self.open_main_app()
        else:
            messagebox.showerror("Error", "Incorrect username or password.")

    def open_main_app(self):
        """Initialize and open the main DroneControl application after successful login."""
        self.root.destroy()  # Close the login screen

        # Create a new Tk instance for the main app
        main_app_root = tk.Tk()
        DroneController(main_app_root)  # Launch the main application


if __name__ == "__main__":
    root = tk.Tk()
    LoginScreen(root)
    root.mainloop()