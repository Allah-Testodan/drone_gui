# -*- coding: utf-8 -*- 
import serial
import tkinter as tk
from tkinter import ttk, messagebox
from tkinter import PhotoImage
import time
import threading
import math

class DroneController:
    def __init__(self, port='COM3', baudrate=9600):
        # Initialize the application with default port and baud rate
        self.port = port
        self.baudrate = baudrate
        
        # Create the main Tkinter window
        self.root = tk.Tk()
        self.root.title("Drone Control")
        self.root.geometry("900x700")  # Set default window size
        self.root.configure(bg="#1C1C1C")  # Set dark theme background

        self.load_icons()  # Load button icons

        # Initialize key variables
        self.status_var = tk.StringVar(value="Idle")
        self.landing_in_progress = False
        self.rotor_speed = tk.DoubleVar(value=0)  # Placeholder for real-time rotor speed
        self.battery_level = tk.DoubleVar(value=0)  # Placeholder for real-time battery level

        self.arduino = None  # Serial communication placeholder
        self.create_gui()  # Create the GUI components
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)  # Handle closing behavior
        self.root.mainloop()  # Start the Tkinter event loop

    def load_icons(self):
        # Load icons for takeoff and land buttons with error handling
        try:
            self.takeoff_icon = PhotoImage(file="takeoff_icon.png").subsample(3, 3)
        except tk.TclError:
            self.takeoff_icon = None  # Default to no icon if not found
        
        try:
            self.land_icon = PhotoImage(file="land_icon.png").subsample(3, 3)
        except tk.TclError:
            self.land_icon = None  # Default to no icon if not found

    def create_gui(self):
        # Set up the main frame for the UI
        main_frame = tk.Frame(self.root, bg="#1C1C1C")
        main_frame.pack(pady=20, padx=20, fill="both", expand=True)

        # Create left and right sections in the main frame
        left_frame = tk.Frame(main_frame, bg="#2B2B2B")
        left_frame.grid(row=0, column=0, sticky="nswe", padx=20)

        right_frame = tk.Frame(main_frame, bg="#2B2B2B")
        right_frame.grid(row=0, column=1, sticky="nswe", padx=20)

        main_frame.columnconfigure(0, weight=2)  # Allocate more space to the left frame
        main_frame.columnconfigure(1, weight=1)  # Right frame takes less space
        main_frame.rowconfigure(0, weight=1)

        # Create control section with buttons and sliders
        self.create_control_section(left_frame)
        # Create gauge section with rotor speed and battery level displays
        self.create_gauge_section(right_frame)

        # Status bar to display connection and operation statuses
        self.status_label = tk.Label(self.root, textvariable=self.status_var, font=("Courier", 14), bg="#1C1C1C", fg="#F2F2F2")
        self.status_label.pack(pady=5)

        # Project label as a footer to personalize the UI
        self.project_label = tk.Label(self.root, text="Projekt Puszczyk", font=("Courier", 10, "italic"), bg="#1C1C1C", fg="#A6A6A6")
        self.project_label.place(anchor='sw', x=20, y=680)

    def create_control_section(self, frame):
        # Label and slider for adjusting and displaying altitude
        self.altitude_label = tk.Label(frame, text="Altitude (cm)", font=("Courier", 14, "bold"), bg="#2B2B2B", fg="#F2F2F2")
        self.altitude_label.pack(pady=5)

        self.altitude_slider = ttk.Scale(frame, from_=0, to=400, orient='horizontal', length=350,
                                         command=self.update_altitude_display, style="TScale")
        self.altitude_slider.pack(pady=10)

        self.current_altitude_label = tk.Label(frame, text="Current Altitude: 0 cm", font=("Courier", 12), bg="#2B2B2B", fg="#F2F2F2")
        self.current_altitude_label.pack(pady=5)

        # Create buttons for control actions (e.g., takeoff, land)
        self.create_control_buttons(frame)

    def create_control_buttons(self, frame):
        # Takeoff button with attached icon and command
        self.takeoff_button = tk.Button(frame, text="Takeoff", image=self.takeoff_icon, compound='left',
                                        command=self.confirm_takeoff, font=("Courier", 12), bg="#3498DB", fg="white",
                                        height=3, width=18, bd=2, activebackground="#2980B9")
        self.takeoff_button.pack(pady=5)

        # Land button with attached icon and command
        self.land_button = tk.Button(frame, text="Land", image=self.land_icon, compound='left', 
                                     command=self.confirm_land, font=("Courier", 12), bg="#E74C3C", fg="white",
                                     height=3, width=18, bd=2, activebackground="#C0392B")
        self.land_button.pack(pady=5)

        # Connect button for initializing Arduino connection
        self.connect_button = tk.Button(frame, text="Connect to Arduino", command=self.confirm_connect,
                                        font=("Courier", 12), bg="#2ECC71", fg="white", height=3, width=18,
                                        bd=2, activebackground="#27AE60")
        self.connect_button.pack(pady=5)

        # Disconnect button to safely close the Arduino connection
        self.disconnect_button = tk.Button(frame, text="Disconnect", command=self.confirm_disconnect,
                                           font=("Courier", 12), bg="#95A5A6", fg="white", height=3, width=18,
                                           bd=2, activebackground="#7F8C8D")
        self.disconnect_button.pack(pady=5)

    def create_gauge_section(self, frame):
        # Create a container for the rotor speed gauge
        rotor_speed_container = tk.Frame(frame, bg="#2B2B2B")
        rotor_speed_container.grid(row=0, column=0, padx=20, pady=15)

        self.rotor_speed_canvas = tk.Canvas(rotor_speed_container, width=250, height=250, bg="#2B2B2B", highlightthickness=0)
        self.rotor_speed_canvas.pack()

        # Label and value display for rotor speed
        self.rotor_speed_label = tk.Label(rotor_speed_container, text="Rotor Speed", font=("Courier", 12, "bold"), bg="#2B2B2B", fg="#E91E63")
        self.rotor_speed_label.pack(pady=5)
        
        self.rotor_speed_value = tk.Label(rotor_speed_container, text="0.0 RPS", font=("Courier", 12), bg="#2B2B2B", fg="#F2F2F2")
        self.rotor_speed_value.pack()

        self.update_rotor_gauge(0)

        # Create a container for the battery level gauge
        battery_container = tk.Frame(frame, bg="#2B2B2B")
        battery_container.grid(row=1, column=0, padx=20, pady=15)

        self.battery_gauge_canvas = tk.Canvas(battery_container, width=250, height=250, bg="#2B2B2B", highlightthickness=0)
        self.battery_gauge_canvas.pack()

        # Label and value display for battery level
        self.battery_label = tk.Label(battery_container, text="Battery Level", font=("Courier", 12, "bold"), bg="#2B2B2B", fg="#00E676")
        self.battery_label.pack(pady=5)
        
        self.battery_value = tk.Label(battery_container, text="0% Battery", font=("Courier", 12), bg="#2B2B2B", fg="#F2F2F2")
        self.battery_value.pack()

        self.update_battery_gauge(0)

    def update_rotor_gauge(self, rps):
        """Update the rotor speed gauge based on the given RPS (rotations per second)."""
        self.rotor_speed_canvas.delete("all")  # Clear previous drawings
        radius = 110
        center_x, center_y = 125, 125

        # Calculate angle for the gauge needle
        angle = (rps / 60.0) * 360
        end_x = center_x + radius * math.cos(math.radians(angle - 90))
        end_y = center_y + radius * math.sin(math.radians(angle - 90))
        self.rotor_speed_canvas.create_line(center_x, center_y, end_x, end_y, fill="#E91E63", width=4)

        # Draw the circular gauge
        self.rotor_speed_canvas.create_oval(center_x - radius, center_y - radius, center_x + radius, center_y + radius, 
                                            outline="#444", width=8)

        self.rotor_speed_value.config(text=f"{rps:.1f} RPS")  # Display the actual RPS value

    def update_battery_gauge(self, percentage):
        """Update the battery level gauge based on the given percentage."""
        self.battery_gauge_canvas.delete("all")
        radius = 110
        center_x, center_y = 125, 125

        # Calculate the arc length for the battery gauge
        angle = (percentage / 100.0) * 360
        self.battery_gauge_canvas.create_arc(center_x - radius, center_y - radius, center_x + radius, center_y + radius,
                                             start=90, extent=-angle, outline="#00E676", style="arc", width=6)

        # Draw the gauge circle
        self.battery_gauge_canvas.create_oval(center_x - radius, center_y - radius, center_x + radius, center_y + radius, 
                                              outline="#444", width=8)

        self.battery_value.config(text=f"{percentage}% Battery")  # Display the battery level

    def update_altitude_display(self, value):
        """Updates the displayed altitude based on the slider value."""
        altitude_cm = int(float(value))
        self.current_altitude_label.config(text=f"Current Altitude: {altitude_cm} cm")
        self.send_altitude(altitude_cm)

    def send_altitude(self, value):
        """Send the altitude setting to the drone via serial communication."""
        if self.arduino:
            altitude_cm = f"{value}\n"
            try:
                self.arduino.write(altitude_cm.encode())
            except Exception as e:
                print(f"Error sending altitude: {e}")

    def confirm_takeoff(self):
        """Prompt the user to confirm the takeoff action."""
        if messagebox.askyesno("Takeoff", "Do you want to initiate takeoff?"):
            self.takeoff()

    def takeoff(self):
        """Send the takeoff command to the drone."""
        if not self.landing_in_progress:
            self.send_command(b'TAKEOFF\n', "Takeoff")

    def confirm_land(self):
        """Prompt the user to confirm the landing action."""
        if messagebox.askyesno("Confirm Landing", "Are you sure you want to land?"):
            self.landing_in_progress = True
            self.initiate_safe_landing()

    def initiate_safe_landing(self):
        """Execute safe landing by adjusting rotor speed based on altitude feedback."""
        if self.arduino:
            try:
                self.update_status("Initiating safe landing...")
                while True:
                    self.arduino.write(b'GET_DISTANCE\n')
                    distance_response = self.arduino.readline().decode().strip()

                    try:
                        distance = float(distance_response)
                        if distance <= 10:  # When close to the ground, land
                            self.send_command(b'LAND\n', "Landed safely")
                            self.update_status("Landed safely.")
                            break
                        else:  # Gradually decrease rotor speed
                            adjusted_speed = max(0, int((distance / 400) * 60))
                            self.send_command(f'SET_SPEED {adjusted_speed}\n'.encode(), f"Adjusting speed to {adjusted_speed} RPS")
                    except ValueError:
                        print(f"Invalid distance reading: {distance_response}")
                    
                    time.sleep(0.3)  # Adjust interval for smooth control
            except Exception as e:
                print(f"Error during landing: {e}")
                self.update_status("Landing error!")

    def confirm_connect(self):
        """Prompt the user to confirm connecting to Arduino."""
        if messagebox.askyesno("Connect to Arduino", "Do you want to connect to the Arduino?"):
            self.connect_to_arduino()

    def confirm_disconnect(self):
        """Prompt the user to confirm disconnecting from Arduino."""
        if messagebox.askyesno("Disconnect", "Are you sure you want to disconnect from the Arduino?"):
            self.disconnect_arduino()

    def connect_to_arduino(self):
        """Initialize connection to Arduino in a separate thread to avoid blocking."""
        threading.Thread(target=self.connect_to_arduino_thread, args=(self.port, self.baudrate), daemon=True).start()

    def connect_to_arduino_thread(self, port, baudrate):
        """Handle serial connection to Arduino, retry if necessary."""
        if self.arduino is None:
            try:
                self.arduino = serial.Serial(port, baudrate, timeout=1)
                self.update_status("Connected to Arduino.")
                self.update_values()  # Start periodic updates
            except serial.SerialException as e:
                self.update_status(f"Error connecting: {e}. Please retry.")
                time.sleep(1)

    def disconnect_arduino(self):
        """Close the serial connection to Arduino."""
        if self.arduino:
            self.arduino.close()
            self.update_status("Disconnected from Arduino.")
            self.arduino = None

    def update_values(self):
        """Periodically request data from the Arduino, such as battery level and rotor speed."""
        if self.arduino:
            try:
                # Fetch battery level from Arduino
                self.arduino.write(b'GET_BATTERY\n')
                battery_response = self.arduino.readline().decode().strip()
                if battery_response:
                    battery_level = int(battery_response)
                    self.smooth_gauge_update(self.battery_level.get(), battery_level, self.update_battery_gauge)
                    self.battery_level.set(battery_level)

                # Fetch rotor speed from Arduino
                self.arduino.write(b'GET_ROTOR_SPEED\n')
                rotor_response = self.arduino.readline().decode().strip()
                if rotor_response:
                    rps = float(rotor_response)
                    self.smooth_gauge_update(self.rotor_speed.get(), rps, self.update_rotor_gauge)
                    self.rotor_speed.set(rps)

                self.root.after(1000, self.update_values)  # Schedule next update

            except Exception as e:
                print(f"Error updating values: {e}")

    def smooth_gauge_update(self, current_value, target_value, update_function):
        """Smoothly transitions to a target value for gauges."""
        step = (target_value - current_value) / 20
        if abs(step) < 0.1:
            step = target_value - current_value

        current_value += step
        if abs(current_value - target_value) > 0.1:
            self.root.after(50, lambda: self.smooth_gauge_update(current_value, target_value, update_function))
        update_function(current_value)

    def send_command(self, command, action_name):
        """Send a command to Arduino and update status."""
        if self.arduino:
            try:
                self.arduino.write(command)
                self.update_status(f"{action_name} initiated.")
            except Exception as e:
                print(f"Error sending {action_name.lower()} command: {e}")

    def update_status(self, message):
        """Update the status display at the bottom of the UI."""
        self.status_var.set(message)
        print(message)

    def on_closing(self):
        """Handle the window close event safely."""
        if self.arduino:
            self.disconnect_arduino()  # Ensure connection is closed
        self.root.destroy()  # Safely close the Tkinter window

if __name__ == "__main__":
    DroneController()
