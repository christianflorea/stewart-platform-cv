import cv2 as cv
import numpy as np
import time


class CameraVision:
    def __init__(self):
        self.cap = cv.VideoCapture(0)
        self.cap.set(cv.CAP_PROP_FPS, 30)
        cv.namedWindow('frame')

        self.current_position_index = 0

        # Ball color range (HSV)
        self.ball_colors = {
            "pingpong": [[15, 100, 100], [25, 255, 255]],
            "bearing": [[0, 0, 0], [179, 255, 50]],
            "golf": [[25, 50, 50], [35, 255, 255]],
        }

        # program types - offsets from center
        self.program_positions = {
            'center': [(0, 0)],
            'square': [(50, 50), (50, -50), (-50, -50), (-50, 50)],
        }

        # Platform color range (HSV)
        self.platform_color = [[110, 150, 91], [116, 250, 200]]

        # Initialize variables to store user selections
        self.ball_type = None
        self.positions = None

        self.platform_center = (0, 0)
        self.last_platform_check = time.time()

        # Get user input for ball type and program number
        self.get_user_input()

    def get_user_input(self):
        # Ask the user for the ball type
        print("Available ball types:")
        for key in self.ball_colors.keys():
            print(f"- {key}")
        while True:
            ball_type = input(
                "Enter the ball type (default: pingpong): ").strip().lower()
            if not ball_type:
                self.ball_type = "pingpong"
                print(f"Default ball type selected: {self.ball_type}")
                break
            elif ball_type in self.ball_colors:
                self.ball_type = ball_type
                print(f"Selected ball type: {self.ball_type}")
                break
            else:
                print("Invalid ball type.")

        # Ask the user for the program number
        print("\nAvailable programs:")
        for key in self.program_positions.keys():
            print(f"- {key}")
        while True:
            program_type = input(
                "Enter the program number (default: center): ").strip().lower()
            if not program_type:
                self.positions = self.program_positions["center"]
                print(f"Default program selected: center")
                break
            elif program_type in self.program_positions:
                self.positions = self.program_positions[program_type]
                print(f"Program: {program_type}")
                break
            else:
                print("Invalid program type.")

    def detect_platform_center(self, platform_contour):
        M = cv.moments(platform_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
        else:
            cx, cy = 0, 0
        return (cx, cy)

    def detect_platform_and_track_ball(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("Failed to grab frame")
                break

            frame = cv.resize(frame, (680, 480))
            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

            # Detect the platform
            platform_lower = np.array(self.platform_color[0])
            platform_upper = np.array(self.platform_color[1])
            platform_mask = cv.inRange(hsv, platform_lower, platform_upper)
            platform_contours, _ = cv.findContours(
                platform_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

            if platform_contours:
                largest_platform = max(platform_contours, key=cv.contourArea)
                # Blue outline
                cv.drawContours(frame, [largest_platform], -1, (255, 0, 0), 2)
                x, y, w, h = cv.boundingRect(largest_platform)
                cv.putText(frame, "Platform Detected", (x, y - 10),
                           cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

                # Calculate the centroid of the platform
                if self.platform_center == (0, 0) or time.time() - self.last_platform_check > 10:
                    self.last_platform_check = time.time()
                    self.platform_center = self.detect_platform_center(
                        largest_platform)
                cx, cy = self.platform_center
                # Green dot at the centroid
                cv.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
                # Display the coordinates of the centroid
                cv.putText(frame, f"Center: ({cx}, {cy})", (cx + 10, cy),
                           cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            else:
                cv.putText(frame, "Platform Not Detected", (10, 30),
                           cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Detect the ball based on the selected ball type
            ball_lower = np.array(self.ball_colors[self.ball_type][0])
            ball_upper = np.array(self.ball_colors[self.ball_type][1])
            ball_mask = cv.inRange(hsv, ball_lower, ball_upper)
            ball_contours, _ = cv.findContours(
                ball_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

            if ball_contours:
                largest_ball = max(ball_contours, key=cv.contourArea)
                ((x_ball, y_ball), radius) = cv.minEnclosingCircle(largest_ball)
                if radius > 10:
                    cv.circle(frame, (int(x_ball), int(y_ball)), int(
                        radius), (0, 165, 255), 2)  # Orange circle
                    cv.circle(frame, (int(x_ball), int(y_ball)),
                              2, (0, 0, 255), -1)  # Red center dot
                    print(
                        f"Ball detected at position: ({int(x_ball)}, {int(y_ball)})")

                    # Track the ball's position
                    x_target, y_target = self.positions[self.current_position_index]
                    cv.circle(frame, (int(x_target), int(y_target)),
                              5, (255, 0, 0), -1)  # Blue dot
                    cv.putText(frame, f"Target: ({int(x_target)}, {int(y_target)})", (10, 30),
                               cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

                    if abs(x_ball - x_target) <= 5 and abs(y_ball - y_target) <= 5:
                        print(
                            f"Ball reached position ({int(x_target)}, {int(y_target)})")
                        self.current_position_index = (
                            self.current_position_index + 1) % len(self.positions)

                else:
                    cv.putText(frame, "Ball not detected", (10, 30),
                               cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                cv.putText(frame, "Ball not detected", (10, 30),
                           cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Display the resulting frame
            cv.imshow('frame', frame)

            # Break the loop when 'q' is pressed
            if cv.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv.destroyAllWindows()


# Instantiate the class and start the program
if __name__ == "__main__":
    tracker = CameraVision()
    tracker.detect_platform_and_track_ball()
