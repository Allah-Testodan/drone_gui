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
        self.port = port
        self.baudrate = baudrate
        
        self.root = tk.Tk()
        self.root.title("Drone Control")
      #  self.root.state('zoomed')  Make the window full-screen 
        self.root.configure(bg="#C0C0C0")  # Classic light gray background
        
        # Attempt to add a window icon, with error handling
        try:
            self.root.iconbitmap("drone_icon.ico")  # Optional: Add a window icon
        except tk.TclError:
            print("Icon file not found. Running without window icon.")

        # Drone status variable
        self.status_var = tk.StringVar(value="Idle")
        self.landing_in_progress = False
        self.rotor_speed = tk.IntVar(value=0)  # Placeholder for rotor speed
        self.battery_level = tk.IntVar(value=0)  # Placeholder for battery level
        
        # Load icons with error handling and resizing
        try:
            self.takeoff_icon = PhotoImage(file="takeoff_icon.png").subsample(3, 3)
        except tk.TclError:
            self.takeoff_icon = None  # Fallback to text only if the icon is missing

        try:
            self.land_icon = PhotoImage(file="land_icon.png").subsample(3, 3)
        except tk.TclError:
            self.land_icon = None  # Fallback to text only if the icon is missing

        # GUI Elements
        self.create_gui()

        # Initialize serial communication
        self.arduino = None

        # Start the GUI loop
        self.root.mainloop()

    def create_gui(self):
        # Main frame that fills the screen
        main_frame = tk.Frame(self.root, bg="#A0A0A0")
        main_frame.pack(pady=20, padx=20, fill="both", expand=True)

        # Split the main frame into left and right
        left_frame = tk.Frame(main_frame, bg="#C0C0C0")
        left_frame.grid(row=0, column=0, sticky="nswe", padx=20)

        right_frame = tk.Frame(main_frame, bg="#C0C0C0")
        right_frame.grid(row=0, column=1, sticky="nswe", padx=20)

        main_frame.columnconfigure(0, weight=2)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(0, weight=1)

        # Altitude controls
        self.altitude_label = tk.Label(left_frame, text="Altitude (cm)", font=("Courier", 14, "bold"), bg="#C0C0C0", fg="black")
        self.altitude_label.pack(pady=10)

        self.altitude_slider = tk.Scale(left_frame, from_=0, to=400, orient='horizontal', length=400,
                                        command=self.update_altitude_display, bg="#D0D0D0", troughcolor="#A9A9A9",
                                        activebackground="#808080", highlightbackground="#404040")
        self.altitude_slider.pack(pady=10)

        self.current_altitude_label = tk.Label(left_frame, text="Current Altitude: 0 cm", font=("Courier", 12, "bold"), bg="#C0C0C0", fg="black")
        self.current_altitude_label.pack(pady=10)

        # Control buttons
        self.takeoff_button = tk.Button(left_frame, text=" Takeoff", image=self.takeoff_icon, compound='left', command=self.confirm_takeoff,
                                        font=("Courier", 12, "bold"), bg="#D0E0E3", fg="black", height=3, width=20, bd=2, activebackground="#A0A0FF")
        self.takeoff_button.pack(pady=10)

        self.land_button = tk.Button(left_frame, text=" Land", image=self.land_icon, compound='left', command=self.confirm_land,
                                     font=("Courier", 12, "bold"), bg="#FFB6C1", fg="black", height=3, width=20, bd=2, activebackground="#FF8C8C")
        self.land_button.pack(pady=10)

        self.connect_button = tk.Button(left_frame, text=" Connect to Arduino", command=self.confirm_connect,
                                        font=("Courier", 12, "bold"), bg="#90EE90", fg="black", height=3, width=20, bd=2, activebackground="#7CCD7C")
        self.connect_button.pack(pady=10)

        self.disconnect_button = tk.Button(left_frame, text=" Disconnect", command=self.confirm_disconnect,
                                           font=("Courier", 12, "bold"), bg="#D3D3D3", fg="black", height=3, width=20, bd=2, activebackground="#A9A9A9")
        self.disconnect_button.pack(pady=10)

        # Gauges section on the right
        self.rotor_speed_canvas = tk.Canvas(right_frame, width=200, height=200, bg="#C0C0C0", highlightthickness=0)
        self.rotor_speed_canvas.grid(row=0, column=0, padx=20, pady=20)
        self.update_rotor_gauge(0)

        self.battery_gauge_canvas = tk.Canvas(right_frame, width=200, height=200, bg="#C0C0C0", highlightthickness=0)
        self.battery_gauge_canvas.grid(row=1, column=0, padx=20, pady=20)
        self.update_battery_gauge(0)

        # Status label at the bottom of the screen
        self.status_label = tk.Label(self.root, textvariable=self.status_var, font=("Courier", 12, "bold"), bg="#A0A0A0", fg="black", width=80)
        self.status_label.pack(pady=10)

    def update_values(self):
        """Periodically request data from the Arduino, such as battery level and rotor speed."""
        if self.arduino:
            try:
                # Request battery status
                self.arduino.write(b'GET_BATTERY\n')
                battery_response = self.arduino.readline().decode().strip()
                if battery_response:
                    battery_level = int(battery_response)
                    self.update_battery_gauge(battery_level)
                    self.battery_level.set(battery_level)

                # Request rotor speed
                self.arduino.write(b'GET_ROTOR_SPEED\n')
                rotor_response = self.arduino.readline().decode().strip()
                if rotor_response:
                    rps = float(rotor_response)
                    self.update_rotor_gauge(rps)

                # Schedule the next update
                self.root.after(1000, self.update_values)

            except Exception as e:
                print(f"Error updating values: {e}")

    def update_altitude_display(self, value):
        """Updates the displayed altitude based on the slider value."""
        altitude_cm = int(float(value))
        self.current_altitude_label.config(text=f"Current Altitude: {altitude_cm} cm")
        self.send_altitude(altitude_cm)

    def update_rotor_gauge(self, rps):
        """Updates the circular gauge to display the rotor speed."""
        self.rotor_speed_canvas.delete("all")
        radius = 80
        center_x, center_y = 100, 100

        # Draw the circular outline
        self.rotor_speed_canvas.create_oval(center_x - radius, center_y - radius, center_x + radius, center_y + radius, outline="#404040", width=4)

        # Draw the needle
        angle = (rps / 60.0) * 360  # Example calculation assuming 60 RPS max
        end_x = center_x + radius * math.cos(math.radians(angle - 90))
        end_y = center_y + radius * math.sin(math.radians(angle - 90))
        self.rotor_speed_canvas.create_line(center_x, center_y, end_x, end_y, fill="#FF0000", width=3)

        # Text Label
        self.rotor_speed_canvas.create_text(center_x, center_y + 90, text=f"{rps:.1f} RPS", font=("Courier", 12, "bold"), fill="black")

    def update_battery_gauge(self, percentage):
        """Updates the circular gauge to display the battery level."""
        self.battery_gauge_canvas.delete("all")
        radius = 80
        center_x, center_y = 100, 100

        # Draw the circular outline
        self.battery_gauge_canvas.create_oval(center_x - radius, center_y - radius, center_x + radius, center_y + radius, outline="#404040", width=4)

        # Fill gauge based on battery level
        angle = (percentage / 100.0) * 360
        self.battery_gauge_canvas.create_arc(center_x - radius, center_y - radius, center_x + radius, center_y + radius,
                                             start=90, extent=-angle, outline="#008000", style="arc", width=5)

        # Text Label
        self.battery_gauge_canvas.create_text(center_x, center_y + 90, text=f"{percentage}% Battery", font=("Courier", 12, "bold"), fill="black")

    def confirm_connect(self):
        if messagebox.askyesno("Connect to Arduino", "Do you want to connect to the Arduino?"):
            self.connect_to_arduino()

    def confirm_disconnect(self):
        if messagebox.askyesno("Disconnect", "Are you sure you want to disconnect from the Arduino?"):
            self.disconnect_arduino()

    def connect_to_arduino(self):
        threading.Thread(target=self.connect_to_arduino_thread, args=(self.port, self.baudrate), daemon=True).start()

    def connect_to_arduino_thread(self, port, baudrate):
        if self.arduino is None:
            try:
                self.arduino = serial.Serial(port, baudrate, timeout=1)
                self.update_status("Connected to Arduino.")
                self.update_values()  # Start periodic updates
            except serial.SerialException as e:
                self.update_status(f"Error connecting: {e}. Please retry.")
                time.sleep(1)

    def disconnect_arduino(self):
        if self.arduino:
            self.arduino.close()
            self.update_status("Disconnected from Arduino.")
            self.arduino = None

    def confirm_takeoff(self):
        if messagebox.askyesno("Takeoff", "Do you want to initiate takeoff?"):
            self.takeoff()

    def takeoff(self):
        if not self.landing_in_progress:
            self.send_command(b'TAKEOFF\n', "Takeoff")

    def confirm_land(self):
        if messagebox.askyesno("Confirm Landing", "Are you sure you want to land?"):
            self.landing_in_progress = True
            self.initiate_safe_landing()

    def send_altitude(self, value):
        if self.arduino:
            altitude_cm = f"{value}\n"
            try:
                self.arduino.write(altitude_cm.encode())
                print(f"Altitude set to: {value} cm")
            except Exception as e:
                print(f"Error sending altitude: {e}")

    def initiate_safe_landing(self):
        if self.arduino:
            try:
                self.update_status("Initiating safe landing...")
                while True:
                    self.arduino.write(b'GET_DISTANCE\n')
                    distance_response = self.arduino.readline().decode().strip()

                    try:
                        distance = float(distance_response)
                        if distance <= 10:
                            self.send_command(b'LAND\n', "Landed safely")
                            self.update_status("Landed safely.")
                            break
                        else:
                            adjusted_speed = max(0, int((distance / 400) * 60))
                            self.send_command(f'SET_SPEED {adjusted_speed}\n'.encode(), f"Adjusting speed to {adjusted_speed} RPS")
                    except ValueError:
                        print(f"Invalid distance reading: {distance_response}")
                    
                    time.sleep(0.3)
            except Exception as e:
                print(f"Error during landing: {e}")
                self.update_status("Landing error!")

    def send_command(self, command, action_name):
        if self.arduino:
            try:
                self.arduino.write(command)
                self.update_status(f"{action_name} initiated.")
                print(f"{action_name} initiated")
            except Exception as e:
                print(f"Error sending {action_name.lower()} command: {e}")

    def update_status(self, message):
        self.status_var.set(message)
        print(message)

if __name__ == "__main__":
    DroneController()
