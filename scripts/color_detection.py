import cv2 as cv
import numpy as np

class HSVViewer:
    def __init__(self):
        self.hsv = None
        self.hsv_values = []  # List to store HSV values from mouse hovers
        self.max_samples = 100  # Number of samples to collect

    # Mouse callback function
    def on_mouse_move(self, event, x, y, flags, param):
        if event == cv.EVENT_MOUSEMOVE:
            if self.hsv is not None:
                hsv_value = self.hsv[y, x]
                h = hsv_value[0]
                s = hsv_value[1]
                v = hsv_value[2]
                self.hsv_values.append((h, s, v))
                # print(f"HSV at ({x}, {y}): ({h}, {s}, {v})")

                # Keep only the last max_samples values
                if len(self.hsv_values) > self.max_samples:
                    self.hsv_values.pop(0)

                if len(self.hsv_values) == self.max_samples:
                    hsv_array = np.array(self.hsv_values)
                    lower_bound = hsv_array.min(axis=0)
                    upper_bound = hsv_array.max(axis=0)
                    print("\nCollected 100 HSV samples.")
                    print(f"Lower HSV bound: {lower_bound}")
                    print(f"Upper HSV bound: {upper_bound}\n")
                    self.hsv_values.clear()

    # Function to display the video feed and collect HSV values
    def run(self):
        # Start capturing video from the webcam
        cap = cv.VideoCapture(0)
        cap.set(cv.CAP_PROP_FPS, 30)
        
        # Create a window and set the mouse callback function
        cv.namedWindow('frame')
        cv.setMouseCallback('frame', self.on_mouse_move)
        
        while True:
            # Read a frame from the webcam
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break
            frame = cv.resize(frame, (680, 480))
            self.hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV) 
            cv.imshow('frame', frame)

            # Break the loop when 'q' is pressed
            if cv.waitKey(1) & 0xFF == ord('q'):
                break

        # Release the capture and close all windows
        cap.release()
        cv.destroyAllWindows()

# Instantiate the class and call the function
viewer = HSVViewer()
viewer.run()
