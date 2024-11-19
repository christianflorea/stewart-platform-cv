import cv2 as cv
import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk
from camera_vision import CameraVision
from smbus2 import SMBus
import threading
from controller import Controller

class CameraVisionGUI:    
    def __init__(self, root):
        self.root = root
        self.root.title("Camera Vision GUI")

        self.cv = CameraVision()
        self.create_widgets()
        self.update_frame()

        self.bus = None
        self.arduino_addr = 0x8
        self.initialize_i2c()

        self.current_servo_positions = [135, 135, 135]  # starting positions

        self.ball_mass_mapping = {
            'golf': 0.04593,
            # NEED TO UPDATE THESE VALUES
            'baering': 0.057,
            'pingpong': 0.0027,
        }

        self.controller = Controller(
            radius=0.035, 
            num_points=100, 
            delay=0.075, 
            kp=1.0, 
            kd=1.0, 
            ki=1.0,
            ball_mass=self.ball_mass_mapping['pingpong']
        )
        self.controller_callback = self.send_servo_commands_bulk
        self.controller_thread = threading.Thread(target=self.controller.run, args=(self.controller_callback,), daemon=True)
        self.controller.active = False

        self.servo_lock = threading.Lock()

    def initialize_i2c(self):
        try:
            self.bus = SMBus(1)  # Use /dev/i2c-1
            print("I2C bus initialized.")
        except Exception as e:
            messagebox.showerror("I2C Communication Error", f"Could not initialize I2C bus: {e}")
            self.bus = None

    def create_widgets(self):
        # 1 ROW x 2 COL
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(0, weight=2)
        self.root.grid_columnconfigure(1, weight=1)

        # Video display panel
        self.video_panel = tk.Label(self.root)
        self.video_panel.grid(row=0, column=0, sticky="nsew")

        # Control panel
        self.control_panel = tk.Frame(self.root, padx=10, pady=10)
        self.control_panel.grid(row=0, column=1, sticky="nsew")

        # Ball Type Selection
        ball_type_frame = tk.LabelFrame(self.control_panel, text="Select Ball Type:", padx=10, pady=10)
        ball_type_frame.pack(fill="x", pady=5)
        self.ball_type_var = tk.StringVar(value=self.cv.ball_type)
        for ball_type in self.cv.ball_colors.keys():
            rb = tk.Radiobutton(ball_type_frame, text=ball_type.capitalize(), variable=self.ball_type_var,
                                value=ball_type, command=self.update_ball_type)
            rb.pack(anchor='w', padx=5, pady=2)

        # Program Selection
        program_frame = tk.LabelFrame(self.control_panel, text="Select Program:", padx=10, pady=10)
        program_frame.pack(fill="x", pady=5)
        self.program_var = tk.StringVar(value=self.cv.program_type)
        for program in self.cv.program_positions.keys():
            rb = tk.Radiobutton(program_frame, text=program.capitalize(), variable=self.program_var,
                                value=program, command=self.update_program)
            rb.pack(anchor='w', padx=5, pady=2)

        # Show/Hide Platform and Ball Contour
        contour_frame = tk.LabelFrame(self.control_panel, text="Contour Options:", padx=10, pady=10)
        contour_frame.pack(fill="x", pady=5)
        self.show_platform_var = tk.BooleanVar(value=True)
        self.show_platform_checkbox = tk.Checkbutton(contour_frame, text="Show Platform Contour",
                                                    variable=self.show_platform_var, command=self.update_show_platform)
        self.show_platform_checkbox.pack(anchor='w', pady=2)

        self.show_ball_var = tk.BooleanVar(value=True)
        self.show_ball_checkbox = tk.Checkbutton(contour_frame, text="Show Ball Contour",
                                                variable=self.show_ball_var, command=self.update_show_ball)
        self.show_ball_checkbox.pack(anchor='w', pady=2)

        # Buttons
        button_frame = tk.Frame(self.control_panel)
        button_frame.pack(fill="x", pady=10)

        self.update_platform_button = ttk.Button(button_frame, text="Update Platform Center", command=self.update_platform_center)
        self.update_platform_button.pack(fill="x", pady=2)

        self.detect_platform_button = ttk.Button(button_frame, text="Start Detection", command=self.start_detect_platform)
        self.detect_platform_button.pack(fill="x", pady=2)

        self.estop_button = ttk.Button(button_frame, text="Stop Detection", command=self.stop_detect_platform)
        self.estop_button.pack(fill="x", pady=2)

        # Homing Button
        self.homing_button = ttk.Button(button_frame, text="Home All Servos", command=self.home_all_servos)
        self.homing_button.pack(fill="x", pady=2)

        # Servo Control
        self.servo_frame = tk.LabelFrame(self.control_panel, text="Servo Controls", padx=10, pady=10)
        self.servo_frame.pack(fill="x", pady=10)
        self.create_servo_controls()

        # Ball position
        position_frame = tk.LabelFrame(self.control_panel, text="Ball Position:", padx=10, pady=10)
        position_frame.pack(fill="x", pady=5)

        self.position_text = tk.Text(position_frame, height=1, width=20)
        self.position_text.pack()
        self.position_text.insert(tk.END, "No data")

        # Controller Controls
        controller_frame = tk.LabelFrame(self.control_panel, text="Path Follower Controls", padx=10, pady=10)
        controller_frame.pack(fill="x", pady=10)

        self.start_controller_button = ttk.Button(controller_frame, text="Start Path Following", command=self.start_controller_following)
        self.start_controller_button.pack(fill="x", pady=2)

        self.stop_controller_button = ttk.Button(controller_frame, text="Stop Path Following", command=self.stop_controller_following)
        self.stop_controller_button.pack(fill="x", pady=2)

    def create_servo_controls(self):
        for i in range(1, 4):
            servo_control_frame = tk.Frame(self.servo_frame)
            servo_control_frame.pack(pady=2)

            tk.Label(servo_control_frame, text=f"Servo {i}").grid(row=0, column=0, columnspan=2)

            up_button = ttk.Button(servo_control_frame, text="Up", command=lambda i=i: self.move_servo_up(i))
            up_button.grid(row=1, column=0, padx=5, pady=2)

            down_button = ttk.Button(servo_control_frame, text="Down", command=lambda i=i: self.move_servo_down(i))
            down_button.grid(row=1, column=1, padx=5, pady=2)

    def move_servo_up(self, servo_number):
        self.move_servo(servo_number, "up")

    def move_servo_down(self, servo_number):
        self.move_servo(servo_number, "down")

    def start_controller_following(self):
        if not self.controller.active:
            self.controller.active = True
            if not self.controller_thread.is_alive():
                self.controller_thread = threading.Thread(target=self.controller.run, args=(self.controller_callback,), daemon=True)
                self.controller_thread.start()
            print("Path following started.")

    def stop_controller_following(self):
        if self.controller.active:
            self.controller.active = False
            print("Path following stopped.")
    
    def send_servo_commands_bulk(self, theta_1, theta_2, theta_3):
        """
        Sends desired servo angles for all three servos in a single I2C command.
        """
        desired_angles = [theta_1, theta_2, theta_3]
        processed_angles = []

        for theta in desired_angles:
            # Convert theta as per original code (270 - theta)
            processed_theta = 270 - theta
            # Clamp the angle between 90 and 180 (adjusted from 85 to 90 for safety)
            processed_theta = max(90, min(180, int(processed_theta)))
            processed_angles.append(processed_theta)

        with self.servo_lock:
            self.current_servo_positions = processed_angles.copy()

        # Prepare the I2C message
        try:
            # Command identifier for bulk command
            command_identifier = 0x10
            # Combine command identifier and angles into a single list
            i2c_data = [command_identifier] + processed_angles
            # Write the data as a block
            self.bus.write_i2c_block_data(self.arduino_addr, 0x00, i2c_data)
            print(f"Bulk Command sent: Set Servos to {processed_angles} degrees.")
        except Exception as e:
            messagebox.showerror("I2C Communication Error", f"Error sending bulk command: {e}")

    def send_servo_commands_automated(self, servo_number, theta):
        """
        Sends desired servo angle for a single servo in an automated manner.
        """
        desired_angle = theta
        processed_theta = 270 - desired_angle
        processed_theta = max(85, min(180, int(processed_theta)))

        with self.servo_lock:
            self.current_servo_positions[servo_number - 1] = processed_theta

        # Prepare the I2C message
        try:
            # Command identifier for automated single servo command
            command_identifier = 0x01
            # Combine command identifier, servo number, and angle
            i2c_data = [command_identifier, servo_number, processed_theta]
            # Write the data as a block
            self.bus.write_i2c_block_data(self.arduino_addr, 0x00, i2c_data)
            print(f"Automated Command sent: Set Servo {servo_number} to {processed_theta} degrees.")
        except Exception as e:
            messagebox.showerror("I2C Communication Error", f"Error sending automated servo command: {e}")

    def move_servo(self, servo_number, direction):
        """
        Sends direction and step commands to the Arduino to move servos manually.
        """
        if self.bus:
            try:
                # Command identifier for manual move command
                command_identifier = 0x00
                servo_byte = servo_number  # 1, 2, or 3
                direction_byte = 0x1 if direction == "up" else 0x0  # 1 for up, 0 for down
                step_byte = 10  # 10 steps
                # Combine into a single list
                i2c_data = [command_identifier, servo_byte, direction_byte, step_byte]
                # Write the data as a block
                self.bus.write_i2c_block_data(self.arduino_addr, 0x00, i2c_data)
                print(f"Manual Move Command sent: Move Servo {servo_number} {direction} by {step_byte} steps.")
            except Exception as e:
                messagebox.showerror("I2C Communication Error", f"Error sending manual move command: {e}")
        else:
            messagebox.showwarning("I2C Bus Not Initialized", "I2C bus is not initialized.")

    def home_all_servos(self):
        if self.bus:
            try:
                # Command identifier for homing command
                # As per updated Arduino code, sending [0x0, 0x0, 0x0]
                # Assuming homing command identifier is 0x00 with servo_byte=0x0
                # Alternatively, if a specific command identifier is needed, adjust accordingly
                command_identifier = 0x00
                servo_byte = 0x0  # Indicates homing
                direction_byte = 0x0  # Not used for homing
                step_byte = 0x0  # Not used for homing
                i2c_data = [command_identifier, servo_byte, direction_byte, step_byte]
                self.bus.write_i2c_block_data(self.arduino_addr, 0x00, i2c_data)
                print("Homing command sent.")

                # Optionally, reset current_servo_positions if homing sets them to known values
                self.current_servo_positions = [135, 135, 135]
            except Exception as e:
                messagebox.showerror("I2C Communication Error", f"Error sending homing command: {e}")
        else:
            messagebox.showwarning("I2C Bus Not Initialized", "I2C bus is not initialized.")

    def update_ball_type(self):
        selected_ball_type = self.ball_type_var.get()
        self.cv.set_ball_type(selected_ball_type)
        print(f"Updated ball type to {selected_ball_type}")
        
        new_mass = self.ball_mass_mapping.get(selected_ball_type.lower(), 0.04593)
        self.controller.set_ball_mass(new_mass)
        print(f"Ball mass set to {new_mass} kg in Controller.")

    def update_program(self):
        selected_program = self.program_var.get()
        self.cv.set_program(selected_program)
        print(f"Updated program to {selected_program}")

    def update_show_platform(self):
        show = self.show_platform_var.get()
        self.cv.set_show_platform_contour(show)
        print(f"Set show_platform_contour to {show}")

    def update_show_ball(self):
        show = self.show_ball_var.get()
        self.cv.set_show_ball_contour(show)
        print(f"Set show_ball_contour to {show}")

    def update_platform_center(self):
        self.cv.update_platform_center()
        print("Platform center updated.")

    def start_detect_platform(self):
        self.cv.start_detection()
        print("Started detection.")

    def stop_detect_platform(self):
        self.cv.stop_detection()
        print("Stopped detection.")

    def update_frame(self):
        # Get the latest frame from CV
        frame = self.cv.get_frame()
        if frame is not None:
            frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
            img = Image.fromarray(frame)
            imgtk = ImageTk.PhotoImage(image=img)
            self.video_panel.imgtk = imgtk
            self.video_panel.configure(image=imgtk)

            if self.cv.ball_position:
                ball_x_px, ball_y_px = self.cv.ball_position
                platform_center_x_px, platform_center_y_px = self.cv.platform_center

                # Normalize coordinates to (0, 0) at platform center
                norm_x = platform_center_x_px - ball_x_px
                norm_y = ball_y_px - platform_center_y_px

                self.controller.set_ball_position(norm_x, norm_y)

                self.position_text.delete('1.0', tk.END)
                self.position_text.insert(tk.END, f"({norm_x}, {norm_y})")
            else:
                self.position_text.delete('1.0', tk.END)
                self.position_text.insert(tk.END, "Ball not detected")

        # Next frame update in 15 ms
        self.root.after(15, self.update_frame)

    def on_closing(self):
        self.cv.release()
        if self.bus:
            self.bus.close()
            print("I2C bus closed.")
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = CameraVisionGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
