import cv2 as cv
import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk
from camera_vision import CameraVision
from smbus2 import SMBus
import threading
from controller import Controller
import time
from enum import Enum

# emum for kp, ki, kd
class PID(Enum):
    KP = "KP"
    KI = "KI"
    KD = "KD"

class CameraVisionGUI:    
    def __init__(self, root):
        self.root = root
        self.root.title("Camera Vision GUI")

        self.cv = CameraVision()

        self.bus = None
        self.arduino_addr = 0x8
        self.initialize_i2c()

        self.current_servo_positions = [110, 110, 110]  # starting positions

        self.ball_mass_mapping = {
            'golf': 0.04593,
            'bearing': 0.057,
            'pingpong': 0.0027,
        }

        self.kp = 0.24
        self.kd = 0.11
        self.ki = 0.001
        self.s = 0.42
        self.controller_step = 0.01
        
        self.pid_history = []
        self.servo_command_idx = 0

        self.controller = Controller(
            delay=0.08, 
            kp=self.kp,
            kd=self.kd,
            ki=self.ki,
            s=self.s,
            ball_mass=self.ball_mass_mapping['golf'],
        )
        self.controller_callback = self.send_servo_commands
        self.controller_thread = threading.Thread(target=self.controller.run, args=(self.controller_callback,), daemon=True)
        self.controller.active = False

        self.servo_lock = threading.Lock()

        self.create_widgets()
        self.update_frame()

    def initialize_i2c(self):
        try:
            self.bus = SMBus(1)  # Use /dev/i2c-1
            print("I2C bus initialized.")
        except Exception as e:
            messagebox.showerror("I2C Communication Error", f"Could not initialize I2C bus: {e}")
            self.bus = None

    def create_widgets(self):
        # Configure the grid layout for the root window
        self.root.grid_rowconfigure(0, weight=2)  # Video display row
        self.root.grid_rowconfigure(1, weight=1)  # PID controls row
        self.root.grid_columnconfigure(0, weight=2)  # Video panel column
        self.root.grid_columnconfigure(1, weight=1)  # Control panel column

        # Video display panel
        self.video_panel = tk.Label(self.root, borderwidth=2, relief="solid")
        self.video_panel.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)

        # PID Controls Panel (under the video)
        pid_panel = ttk.Frame(self.root, padding=10)
        pid_panel.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)
        
        # Configure grid for pid_panel to have 3 columns
        pid_panel.columnconfigure(0, weight=1, uniform="pid")
        pid_panel.columnconfigure(1, weight=1, uniform="pid")
        pid_panel.columnconfigure(2, weight=1, uniform="pid")
        pid_panel.rowconfigure(0, weight=1)

        # Kp Controller
        Kp_controller_frame = ttk.LabelFrame(pid_panel, text="Kp", padding=10)
        Kp_controller_frame.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        
        self.Kp_text = ttk.Label(Kp_controller_frame, text=f"Kp: {self.kp}", anchor="center")
        self.Kp_text.pack(fill="x", pady=(0, 5))
        
        Kp_buttons_frame = ttk.Frame(Kp_controller_frame)
        Kp_buttons_frame.pack(fill="x", pady=(0, 5))
        
        ttk.Button(Kp_buttons_frame, text="▲", command=lambda: self.change_value(PID.KP, self.controller_step)).pack(side="left", expand=True, fill="x", padx=(0, 2))
        ttk.Button(Kp_buttons_frame, text="▼", command=lambda: self.change_value(PID.KP, -self.controller_step)).pack(side="right", expand=True, fill="x", padx=(2, 0))

        # Ki Controller
        Ki_controller_frame = ttk.LabelFrame(pid_panel, text="Ki", padding=10)
        Ki_controller_frame.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")
        
        self.Ki_text = ttk.Label(Ki_controller_frame, text=f"Ki: {self.ki}", anchor="center")
        self.Ki_text.pack(fill="x", pady=(0, 5))
        
        Ki_buttons_frame = ttk.Frame(Ki_controller_frame)
        Ki_buttons_frame.pack(fill="x", pady=(0, 5))
        
        ttk.Button(Ki_buttons_frame, text="▲", command=lambda: self.change_value(PID.KI, self.controller_step / 10)).pack(side="left", expand=True, fill="x", padx=(0, 2))
        ttk.Button(Ki_buttons_frame, text="▼", command=lambda: self.change_value(PID.KI, -self.controller_step / 10)).pack(side="right", expand=True, fill="x", padx=(2, 0))

        # Kd Controller
        Kd_controller_frame = ttk.LabelFrame(pid_panel, text="Kd", padding=10)
        Kd_controller_frame.grid(row=0, column=2, padx=5, pady=5, sticky="nsew")
        
        self.Kd_text = ttk.Label(Kd_controller_frame, text=f"Kd: {self.kd}", anchor="center")
        self.Kd_text.pack(fill="x", pady=(0, 5))
        
        Kd_buttons_frame = ttk.Frame(Kd_controller_frame)
        Kd_buttons_frame.pack(fill="x", pady=(0, 5))
        
        ttk.Button(Kd_buttons_frame, text="▲", command=lambda: self.change_value(PID.KD, self.controller_step)).pack(side="left", expand=True, fill="x", padx=(0, 2))
        ttk.Button(Kd_buttons_frame, text="▼", command=lambda: self.change_value(PID.KD, -self.controller_step)).pack(side="right", expand=True, fill="x", padx=(2, 0))

        # Control panel (right column)
        self.control_panel = ttk.Frame(self.root, padding=10)
        self.control_panel.grid(row=0, column=1, rowspan=2, sticky="nsew", padx=5, pady=5)

        # Ball Type Selection
        ball_type_frame = ttk.LabelFrame(self.control_panel, text="Select Ball Type", padding=10)
        ball_type_frame.pack(fill="x", pady=5)
        self.ball_type_var = tk.StringVar(value=self.cv.ball_type)
        for ball_type in self.cv.ball_colors.keys():
            rb = ttk.Radiobutton(
                ball_type_frame,
                text=ball_type.capitalize(),
                variable=self.ball_type_var,
                value=ball_type,
                command=self.update_ball_type,
            )
            rb.pack(anchor="w", padx=5, pady=2)

        # Program Selection
        program_frame = ttk.LabelFrame(self.control_panel, text="Select Program", padding=10)
        program_frame.pack(fill="x", pady=5)
        self.program_var = tk.StringVar(value=self.cv.program_type)
        for program in self.cv.program_positions.keys():
            rb = ttk.Radiobutton(
                program_frame,
                text=program.capitalize(),
                variable=self.program_var,
                value=program,
                command=self.update_program,
            )
            rb.pack(anchor="w", padx=5, pady=2)

        # Contour Options
        contour_frame = ttk.LabelFrame(self.control_panel, text="Contour Options", padding=10)
        contour_frame.pack(fill="x", pady=5)
        self.show_platform_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            contour_frame, text="Show Platform Contour", variable=self.show_platform_var, command=self.update_show_platform
        ).pack(anchor="w", pady=2)
        self.show_ball_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            contour_frame, text="Show Ball Contour", variable=self.show_ball_var, command=self.update_show_ball
        ).pack(anchor="w", pady=2)

        # Buttons
        button_frame = ttk.Frame(self.control_panel, padding=10)
        button_frame.pack(fill="x", pady=10)
        ttk.Button(button_frame, text="Initialize", command=self.initialize_platform).pack(fill="x", pady=2)
        ttk.Button(button_frame, text="Home All Servos", command=self.home_all_servos).pack(fill="x", pady=2)
        ttk.Button(button_frame, text="Update Platform Center", command=self.update_platform_center).pack(fill="x", pady=2)
        ttk.Button(button_frame, text=f"{'Stop' if self.cv.is_detecting() else 'Start'} Detection", command=self.toggle_detect_platform).pack(fill="x", pady=2)
        ttk.Button(button_frame, text=f"{'Stop' if self.controller.is_active() else 'Start'} Path Follow", command=self.toggle_controller_following).pack(fill="x", pady=2)

        # Ball Position Display
        ball_position_frame = ttk.LabelFrame(self.control_panel, text="Ball Position", padding=10)
        ball_position_frame.pack(fill="x", pady=10)

        self.ball_position_text = tk.Text(ball_position_frame, height=1, width=20)
        self.ball_position_text.pack(fill="x", pady=5)
        self.ball_position_text.insert("1.0", "No data")  # Default text
        
        # Servo Control
        self.servo_frame = tk.LabelFrame(self.control_panel, text="Servo Controls", padx=10, pady=10)
        self.servo_frame.pack(fill="x", pady=10)
        self.create_servo_controls()


    def initialize_platform(self):
        """
        Home, update platform center, start detection, and reset the controller.
        """
        
        self.stop_detection()
        time.sleep(0.1)
        self.home_all_servos()
        time.sleep(2)
        self.cv.start_detection()

        # Reset the existing controller with updated parameters
        
        self.controller.reset()
        self.controller.set_kp(self.kp)
        self.controller.set_ki(self.ki)
        self.controller.set_kd(self.kd)
        self.controller.set_ball_mass(self.ball_mass_mapping['golf'])
        
        self.update_platform_center()
        self.toggle_controller_following()

        print("Platform initialized with updated controller parameters.")

    def create_servo_controls(self):
        for i in range(1, 4):
            servo_control_frame = tk.Frame(self.servo_frame)
            servo_control_frame.pack(pady=2)

            tk.Label(servo_control_frame, text=f"Servo {i}").grid(row=0, column=0, columnspan=2)

            up_button = ttk.Button(servo_control_frame, text="Up", command=lambda i=i: self.move_servo(i, "up"))
            up_button.grid(row=1, column=0, padx=5, pady=2)

            down_button = ttk.Button(servo_control_frame, text="Down", command=lambda i=i: self.move_servo(i, "down"))
            down_button.grid(row=1, column=1, padx=5, pady=2)

    def send_servo_commands(self, theta_1, theta_2, theta_3):
        """
        Receives desired servo angles from Controller and sends I2C commands.
        """
        start_time = time.time()
        self.servo_command_idx = (self.servo_command_idx + 1) % 1
        
        if self.servo_command_idx:
            return
        
        desired_angles = [theta_1, theta_2, theta_3]
        for i in range(3):
            desired_angles[i] = 270 - desired_angles[i]
            print('angles: ', desired_angles)
            desired_angles[i] = max(90, min(180, int(desired_angles[i])))
            with self.servo_lock:
                self.current_servo_positions[i] = desired_angles[i]
        
#         print(f"Sending bulk servo angles: {desired_angles}")

        # Send I2C command
        if self.bus:
            try:
                self.bus.write_i2c_block_data(self.arduino_addr, 4, [a for a in desired_angles])
                end_time = time.time()
#                 print(f"I2C command took {end_time - start_time} seconds.")
            except Exception as e:
                messagebox.showerror("I2C Communication Error", f"Error sending command: {e}")
        else:
            messagebox.showwarning("I2C Bus Not Initialized", "I2C bus is not initialized.")


    def move_servo(self, servo_number, direction):
        """
        Sends direction and step commands to the Arduino to move servos manually.
        """
        if self.bus:
            try:
                servo_byte = servo_number  # 1, 2, or 3
                direction_byte = 0x1 if direction == "up" else 0x0  # 1 for up, 0 for down
                step_byte = 0xA  # 10 steps
                self.bus.write_i2c_block_data(self.arduino_addr, servo_byte, [direction_byte, step_byte])
                print(f"Command sent: Move Servo {servo_number} {direction} by {step_byte} steps.")
            except Exception as e:
                messagebox.showerror("I2C Communication Error", f"Error sending command: {e}")
        else:
            messagebox.showwarning("I2C Bus Not Initialized", "I2C bus is not initialized.")

    def home_all_servos(self):
        if self.bus:
            try:
                # home: servo_byte = 0x0
                # direction_byte = 0x0, step_byte = 0x0 (these do not matter)
                servo_byte = 0x0
                direction_byte = 0x0
                step_byte = 0x0
                self.bus.write_i2c_block_data(self.arduino_addr, servo_byte, [direction_byte, step_byte])
                print("Homing command sent.")

                self.current_servo_positions = [145, 145, 145]
            except Exception as e:
                messagebox.showerror("I2C Communication Error", f"Error sending homing command: {e}")
        else:
            messagebox.showwarning("I2C Bus Not Initialized", "I2C bus is not initialized.")
            
    def stop_detection(self):
        self.controller.active = False
        self.cv.stop_detection()
    
    def toggle_controller_following(self):
        if not self.controller.active:
            self.controller.active = True
            if not self.controller_thread.is_alive():
                self.controller_thread = threading.Thread(target=self.controller.run, args=(self.controller_callback,), daemon=True)
                self.controller_thread.start()
            print("Path following started.")
        else:
            self.controller.active = False
            print("Path following stopped.")
    
    def change_value(self, pid_var, step):
        if pid_var == PID.KP:
            self.kp += step
            self.controller.set_kp(self.kp)
            self.Kp_text.config(text=f"Kp: {self.kp}")
            print(f"Kp changed to {self.kp}")
        elif pid_var == PID.KI:
            self.ki += step
            self.controller.set_ki(self.ki)
            self.Ki_text.config(text=f"Ki: {self.ki}")
            print(f"Ki changed to {self.ki}")
        elif pid_var == PID.KD:
            self.kd += step
            self.controller.set_kd(self.kd)
            self.Kd_text.config(text=f"Kd: {self.kd}")
            print(f"Kd changed to {self.kd}")
        else:
            print("Invalid PID variable.")

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

    def toggle_detect_platform(self):
        if self.cv.is_detecting():
            self.cv.stop_detection()
            print("Stopped detection.")

        else:
            self.cv.start_detection()
            print("Started detection.")

    def update_frame(self):
        # Get the latest frame from CV
        frame = self.cv.get_frame()
        if frame is not None:
            # Retrieve x_pid and y_pid from the controller
            x_pid, y_pid = self.controller.get_pid()

            # Define a scale factor for plotting (adjust as needed)
            scale = 24200/25  # Pixels per unit, adjust based on your application's scale

            # Determine the center of the platform in pixels
            if self.cv.platform_center:
                center_x, center_y = self.cv.platform_center
            else:
                center_x, center_y = self.cv.frame_width // 2, self.cv.frame_height // 2  # Default center

            # Map PID values to pixel positions
            plot_x = int(center_x - x_pid * scale)
            plot_y = int(center_y + y_pid * scale)  # Invert y-axis for image coordinates

            # Draw a circle representing the PID position
            cv.circle(frame, (plot_x, plot_y), 10, (0, 255, 255), -1)  # Yellow dot

            # Optionally, draw a line from center to PID position
            cv.line(frame, (center_x, center_y), (plot_x, plot_y), (0, 255, 255), 2)
            
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

                norm_x = norm_x * round(250/242, 3)
                norm_y = norm_y * round(250/242, 3)

                self.controller.set_ball_position(norm_x, norm_y)

                self.ball_position_text.delete('1.0', tk.END)
                self.ball_position_text.insert(tk.END, f"({norm_x}, {norm_y})")
            else:
                self.ball_position_text.delete('1.0', tk.END)
                self.ball_position_text.insert(tk.END, "Ball not detected")

            # Update controller's goal position
            if self.cv.current_target_position and self.cv.platform_center:
                target_x_px, target_y_px = self.cv.current_target_position
                platform_center_x_px, platform_center_y_px = self.cv.platform_center

                # Normalize coordinates to (0, 0) at platform center
                norm_x_goal = platform_center_x_px - target_x_px
                norm_y_goal = target_y_px - platform_center_y_px

                norm_x_goal = norm_x_goal * round(250/242, 3)
                norm_y_goal = norm_y_goal * round(250/242, 3)

                self.controller.set_goal_position(norm_x_goal, norm_y_goal)
            
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
