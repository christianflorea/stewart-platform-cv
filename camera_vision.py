import cv2 as cv
import numpy as np
import time
import threading

PLATFORM_COLOUR = [[110, 140, 80], [120, 255, 180]]

class CameraVision:
    def __init__(self):
        # initialize OpenCV
        self.cap = cv.VideoCapture(0)
        self.cap.set(cv.CAP_PROP_FPS, 30)
        ret, frame = self.cap.read()
        if not ret:
            print("Failed to grab frame")
            exit()
        self.frame_height, self.frame_width = frame.shape[:2]

        self.current_position_index = 0
        self.ball_position = None
        self.platform_center = None

        # Ball color range (HSV)
        self.ball_colors = {
            "pingpong": [[16, 90, 160], [24, 170, 230]],
            "bearing": [[0, 0, 0], [179, 255, 50]],
            "golf": [[40, 10, 150], [100, 100, 255]],
        }
        self.ball_type = "pingpong"  # Default ball type
        self.ball_lower = np.array(self.ball_colors[self.ball_type][0])
        self.ball_upper = np.array(self.ball_colors[self.ball_type][1])

        # Program positions - offsets from center
        self.program_positions = {
            'center': [(0, 0)],
            'square': [(50, 50), (50, -50), (-50, -50), (-50, 50)],
        }
        self.program_type = 'center'
        self.positions = self.program_positions[self.program_type]

        self.platform_circle_center = (0, 0)
        self.platform_circle_radius = 0
        self.show_platform_circle = False

        # Platform color range (HSV)
        self.platform_color = PLATFORM_COLOUR

        # contour visibility
        self.show_platform_contour = True
        self.show_ball_contour = True

        # contours
        self.platform_contour = None
        self.ball_contour = None

        # flag to control the detection
        self.detecting = False

        # thread synchronization
        self.lock = threading.Lock()

        self.capturing = True
        threading.Thread(target=self.capture_frames, daemon=True).start()

    def capture_frames(self):
        while self.capturing:
            ret, frame = self.cap.read()
            if not ret:
                print("Failed to grab frame")
                break

            frame = cv.resize(frame, (self.frame_width, self.frame_height))
            with self.lock:
                self.frame = frame.copy()
            time.sleep(0.05)

    def set_ball_type(self, ball_type):
        with self.lock:
            if ball_type in self.ball_colors:
                self.ball_type = ball_type
                self.ball_lower = np.array(self.ball_colors[self.ball_type][0])
                self.ball_upper = np.array(self.ball_colors[self.ball_type][1])
                print(f"Ball type set to: {self.ball_type}")
            else:
                print(f"Ball type '{ball_type}' not recognized. Using default.")

    def set_program(self, program_type):
        with self.lock:
            if program_type in self.program_positions:
                self.program_type = program_type
                self.positions = self.program_positions[program_type]
                self.current_position_index = 0
                print(f"Program set to: {program_type}")
            else:
                print(f"Program '{program_type}' not recognized. Using default.")

    def set_show_platform_contour(self, show):
        with self.lock:
            self.show_platform_contour = show
            if not show:
                self.show_platform_circle = False

    def set_show_ball_contour(self, show):
        with self.lock:
            self.show_ball_contour = show

    def detect_platform_center(self, platform_contour):
        M = cv.moments(platform_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
        else:
            cx, cy = 0, 0
        return (cx, cy)

    def update_platform_center(self):
        with self.lock:
            if hasattr(self, 'frame'):
                frame = self.frame.copy()
            else:
                print("No frame available to detect platform.")
                return

        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        # Detect the platform
        platform_lower = np.array(self.platform_color[0])
        platform_upper = np.array(self.platform_color[1])
        platform_mask = cv.inRange(hsv, platform_lower, platform_upper)
        platform_contours, _ = cv.findContours(
            platform_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        with self.lock:
            if platform_contours:
                largest_platform = max(platform_contours, key=cv.contourArea)
                self.platform_contour = largest_platform

                self.platform_center = self.detect_platform_center(largest_platform)
                print(f"Platform center updated: {self.platform_center}")

                # Calculate minimum enclosing circle
                (x, y), radius = cv.minEnclosingCircle(largest_platform)
                self.platform_circle_center = (int(x), int(y))
                self.platform_circle_radius = int(radius)
                self.show_platform_circle = True

                print(f"Platform enclosing circle radius: {self.platform_circle_radius} pixels")
            else:
                self.platform_contour = None
                self.platform_center = None
                # Reset enclosing circle properties
                self.platform_circle_center = (0, 0)
                self.platform_circle_radius = 0
                self.show_platform_circle = False
                print("Platform not detected.")

    def is_detecting(self):
        return self.detecting
    
    def start_detection(self):
        if not self.detecting:
            self.detecting = True
            threading.Thread(target=self.detect_platform_and_track_ball, daemon=True).start()
            print("Started detection.")

    def stop_detection(self):
        self.detecting = False
        print("Stopped detection.")

    def detect_platform_and_track_ball(self):
        while self.detecting:
            with self.lock:
                if hasattr(self, 'frame'):
                    frame = self.frame.copy()
                    platform_contour = self.platform_contour.copy() if self.platform_contour is not None else None
                else:
                    continue

            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

            with self.lock:
                if platform_contour is not None:
                    platform_mask = np.zeros_like(frame[:, :, 0])
                    cv.drawContours(platform_mask, [platform_contour], -1, 255, thickness=cv.FILLED)

                    ball_mask = cv.inRange(hsv, self.ball_lower, self.ball_upper)

                    # make sure we are only looking for the ball within the platform
                    masked_ball = cv.bitwise_and(ball_mask, ball_mask, mask=platform_mask)

                    ball_contours, _ = cv.findContours(
                        masked_ball, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

                    if ball_contours:
                        largest_ball = max(ball_contours, key=cv.contourArea)
                        self.ball_contour = largest_ball

                        ((x_ball, y_ball), radius) = cv.minEnclosingCircle(largest_ball)
                        if radius > 10:
                            self.ball_position = (int(x_ball), int(y_ball))

                            if self.platform_center:
                                x_inc, y_inc = self.positions[self.current_position_index]
                                x_target = self.platform_center[0] + x_inc
                                y_target = self.platform_center[1] + y_inc

                                if abs(x_ball - x_target) <= 5 and abs(y_ball - y_target) <= 5:
                                    print(f"Ball reached position ({int(x_target)}, {int(y_target)})")
                                    self.current_position_index = (
                                        self.current_position_index + 1) % len(self.positions)
                        else:
                            self.ball_position = None
                            self.ball_contour = None
                    else:
                        self.ball_position = None
                        self.ball_contour = None
                else:
                    print("Platform contour not available. Cannot detect ball within platform.")
                    self.ball_position = None
                    self.ball_contour = None

            time.sleep(0.03)

    def get_frame(self):
        with self.lock:
            if hasattr(self, 'frame'):
                frame = self.frame.copy()

                # Draw platform contour
                if self.show_platform_contour and self.platform_contour is not None:
                    cv.drawContours(frame, [self.platform_contour], -1, (255, 0, 0), 2)  # Blue contour

                # Draw platform center
                if self.platform_center:
                    cv.circle(frame, self.platform_center, 5, (0, 255, 0), -1)  # Green center
                    cv.putText(frame, f"Center: {self.platform_center}", 
                            (self.platform_center[0] + 10, self.platform_center[1]),
                            cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                # Draw ball contour
                if self.show_ball_contour and self.ball_contour is not None:
                    cv.drawContours(frame, [self.ball_contour], -1, (0, 165, 255), 2)  # Orange contour

                # Draw ball position
                if self.ball_position:
                    cv.circle(frame, self.ball_position, 5, (0, 0, 255), -1)  # Red ball
                    cv.putText(frame, f"Ball: {self.ball_position}", 
                            (self.ball_position[0] + 10, self.ball_position[1]),
                            cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

                # Draw target position
                if self.platform_center and self.positions:
                    x_inc, y_inc = self.positions[self.current_position_index]
                    x_target = self.platform_center[0] + x_inc
                    y_target = self.platform_center[1] + y_inc
                    cv.circle(frame, (int(x_target), int(y_target)), 5, (255, 0, 0), -1)  # Blue target
                    cv.putText(frame, f"Target: ({int(x_target)}, {int(y_target)})", (10, 30),
                            cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

                # Draw platform enclosing circle
                if self.show_platform_circle:
                    cv.circle(frame, self.platform_circle_center, self.platform_circle_radius, (0, 0, 255), 2)  # Red circle
                    # Display the radius value near the circle
                    cv.putText(frame, f"Radius: {self.platform_circle_radius} px", 
                            (self.platform_circle_center[0] + self.platform_circle_radius + 10, 
                                self.platform_circle_center[1]),
                            cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

                return frame
            else:
                return None

    def release(self):
        self.detecting = False
        self.capturing = False
        self.cap.release()
        print("Camera released.")
