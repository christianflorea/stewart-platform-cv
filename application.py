import cv2 as cv
import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk
from camera_vision import CameraVision
import time
from smbus2 import SMBus

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

    def create_servo_controls(self):
        for i in range(1, 4):
            servo_control_frame = tk.Frame(self.servo_frame)
            servo_control_frame.pack(pady=2)

            tk.Label(servo_control_frame, text=f"Servo {i}").grid(row=0, column=0, columnspan=2)

            up_button = ttk.Button(servo_control_frame, text="Up", command=lambda i=i: self.move_servo(i, "up"))
            up_button.grid(row=1, column=0, padx=5, pady=2)

            down_button = ttk.Button(servo_control_frame, text="Down", command=lambda i=i: self.move_servo(i, "down"))
            down_button.grid(row=1, column=1, padx=5, pady=2)

    def move_servo(self, servo_number, direction):
        if self.bus:
            try:
                servo_byte = servo_number  # 1, 2, or 3
                direction_byte = 0x1 if direction == "up" else 0x0  # 1 for up, 0 for down
                # Send data to Arduino
                self.bus.write_i2c_block_data(self.arduino_addr, servo_byte, [direction_byte])
                print(f"Command sent: Move Servo {servo_number} {direction}.")
            except Exception as e:
                messagebox.showerror("I2C Communication Error", f"Error sending command: {e}")
        else:
            messagebox.showwarning("I2C Bus Not Initialized", "I2C bus is not initialized.")


    def home_all_servos(self):
        if self.bus:
            try:
                # Define homing command: servo_byte = 0x0, direction_byte = 0x0
                servo_byte = 0x0
                direction_byte = 0x0
                self.bus.write_i2c_block_data(self.arduino_addr, servo_byte, [direction_byte])
                print("Homing command sent.")
            except Exception as e:
                messagebox.showerror("I2C Communication Error", f"Error sending homing command: {e}")
        else:
            messagebox.showwarning("I2C Bus Not Initialized", "I2C bus is not initialized.")


    def update_ball_type(self):
        selected_ball_type = self.ball_type_var.get()
        self.cv.set_ball_type(selected_ball_type)
        print(f"Updated ball type to {selected_ball_type}")

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

            # Update ball position in the text box
            if self.cv.ball_position:
                self.position_text.delete('1.0', tk.END)
                self.position_text.insert(tk.END, f"{self.cv.ball_position}")
            else:
                self.position_text.delete('1.0', tk.END)
                self.position_text.insert(tk.END, "Ball not detected")

        # Next frame update in 15 ms
        self.root.after(15, self.update_frame)

    def on_closing(self):
        # Release resources and close the window
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
