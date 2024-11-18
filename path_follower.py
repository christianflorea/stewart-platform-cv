import numpy as np
import time
import threading

class Controller:
    """
    Controller class to follow the ball's position using PID control.

    Attributes:
        radius (float): Radius of the circular path in meters.
        num_points (int): Number of points in the path.
        delay (float): Time delay between each point in seconds.
        kp (float): Proportional gain for PID controller.
        kd (float): Derivative gain for PID controller.
        ki (float): Integral gain for PID controller.
        ball_mass (float): Mass of the ball in kg.
    """

    def __init__(self, radius=0.035, num_points=100, delay=0.075, kp=1.0, kd=1.0, ki=1.0, ball_mass=0.04593):
        self.radius = radius
        self.num_points = num_points
        self.delay = delay
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.ball_mass = ball_mass

        # Placeholder for current ball position, initialized at center
        self.x_ball = 0.0
        self.y_ball = 0.0

        # PID controller state
        self.error_x_last = 0.0
        self.error_y_last = 0.0
        self.integral_x = 0.0
        self.integral_y = 0.0

        # Constants
        self.g = 9.81  # Acceleration due to gravity (m/s^2)
        self.MAX_PID = 0.10206  # Maximum PID output

        # Storage for results
        self.results = {
            'V': [],
            'theta': [],
            'phi': [],
            'alpha': [],
            'beta': [],
            'gamma': [],
            'theta_1': [],
            'theta_2': [],
            'theta_3': []
        }

        self.active = False

        self.lock = threading.Lock()

    def set_ball_mass(self, mass):
        """
        Sets the mass of the ball.
        """
        with self.lock:
            self.ball_mass = mass
        print(f"Ball mass set to {self.ball_mass} kg.")

    def set_ball_position(self, x, y):
        """
        Sets the current ball position.
        """
        with self.lock:
            self.x_ball = x
            self.y_ball = y
        print(f"Ball position updated to ({self.x_ball}, {self.y_ball})")

    def run(self, callback):
        """
        Executes the path following and PID control loop.
        """
        print("Starting path following and PID control...")
        print("------------------------------------------")
        self.active = True
        start_time = time.perf_counter()

        while self.active:
            with self.lock:
                # Current position of the ball
                x_ball = self.x_ball
                y_ball = self.y_ball

            # Goal position (platform center)
            x_goal = 0.0
            y_goal = 0.0

            # Calculate errors
            error_x = x_goal - x_ball
            error_y = y_goal - y_ball

            # Time management
            current_time = time.perf_counter()
            if hasattr(self, 'last_time'):
                time_between = current_time - self.last_time
            else:
                time_between = 0.0  # No previous time
            self.last_time = current_time

            # Derivative calculation
            if hasattr(self, 'error_x_last') and time_between > 0:
                derivative_x = (error_x - self.error_x_last) / time_between
                derivative_y = (error_y - self.error_y_last) / time_between
            else:
                derivative_x = 0.0
                derivative_y = 0.0

            # Integral calculation
            self.integral_x += error_x * time_between
            self.integral_y += error_y * time_between

            # PID output
            with self.lock:
                kp = self.kp
                kd = self.kd
                ki = self.ki
                ball_mass = self.ball_mass

            x_pid = (kp * error_x) + (kd * derivative_x) + (ki * self.integral_x)
            y_pid = (kp * error_y) + (kd * derivative_y) + (ki * self.integral_y)

            # Update last errors
            self.error_x_last = error_x
            self.error_y_last = error_y

            # Clamp PID outputs
            x_pid = np.clip(x_pid, -self.MAX_PID, self.MAX_PID)
            y_pid = np.clip(y_pid, -self.MAX_PID, self.MAX_PID)

            # Calculate velocity and angles
            V = np.sqrt(x_pid**2 + y_pid**2)
            theta_rad = np.arctan2(y_pid, x_pid)
            theta_deg = np.degrees(theta_rad)

            # Calculate phi based on dynamics (assuming some relation)
            try:
                phi_rad = np.arcsin((2 * V) / (ball_mass * self.g)) / 2
                phi_deg = np.degrees(phi_rad)
                phi_deg = np.clip(phi_deg, -15, 15)
            except ValueError:
                # Handle domain error if (2*V)/(m_ball*g) > 1
                phi_deg = 15 if V > 0 else -15

            # Calculate velocity components
            V_x = abs(V * np.cos(np.radians(theta_deg)) / np.cos(np.radians(phi_deg)))
            V_y = abs(V * np.sin(np.radians(theta_deg)) / np.cos(np.radians(phi_deg)))

            # Calculate phi_x and phi_y
            phi_x_deg = np.degrees(np.arccos(x_pid / V_x)) if V_x != 0 else 0.0
            phi_y_deg = np.degrees(np.arccos(y_pid / V_y)) if V_y != 0 else 0.0

            # Adjust V_x and V_y based on quadrant
            if x_pid != 0 and y_pid != 0:
                if x_pid > 0 and y_pid > 0:  # Quadrant 1
                    V_x = -V_x
                    V_y = -V_y
                elif x_pid > 0 and y_pid < 0:  # Quadrant 2
                    V_x = -V_x
                    # V_y remains
                elif x_pid < 0 and y_pid < 0:  # Quadrant 3
                    # V_x and V_y remain
                    pass
                elif x_pid < 0 and y_pid > 0:  # Quadrant 4
                    # V_x remains
                    V_y = -V_y

            # Calculate Normal Vector components
            N_x = -V_x * V_y * np.cos(np.radians(phi_y_deg)) * np.sin(np.radians(phi_x_deg))
            N_y = V_x * V_y * np.cos(np.radians(phi_x_deg)) * np.sin(np.radians(phi_y_deg))
            N_z = V_x * V_y * np.cos(np.radians(phi_x_deg)) * np.cos(np.radians(phi_y_deg))

            # Calculate alpha, beta, gamma
            alpha = abs(N_x)
            beta = abs(N_y)
            gamma = abs(N_z)

            # Adjust signs based on quadrant
            if x_pid > 0 and y_pid > 0:  # Quadrant 1
                alpha = -alpha
                beta = -beta
            elif x_pid > 0 and y_pid < 0:  # Quadrant 2
                alpha = -alpha
            elif x_pid < 0 and y_pid < 0:  # Quadrant 3
                # All three are positive
                pass
            elif x_pid < 0 and y_pid > 0:  # Quadrant 4
                beta = -beta

            # Constants for Ball Joint Position Calculations
            L = 0.10206  # Distance to plate center in meters
            h = 121.922 / 1000  # Height in meters

            # Leg 1 Calculations
            try:
                x1 = np.sqrt(L**2 - (h - (h - (L * alpha) / np.sqrt(gamma**2 + alpha**2)))**2)
            except ValueError:
                x1 = 0.0  # Handle domain error

            y1 = 0.0
            z1 = h - ((L * alpha) / np.sqrt(gamma**2 + alpha**2))

            # Leg 2 Calculations
            denominator_leg2 = np.sqrt(4 * gamma**2 + alpha**2 - 2 * np.sqrt(3) * alpha * beta + 3 * beta**2)
            if denominator_leg2 != 0:
                x2 = (-L * gamma) / denominator_leg2
                y2 = (np.sqrt(3) * L * gamma) / denominator_leg2
                z2 = h + ((L * (alpha - np.sqrt(3) * beta)) / denominator_leg2)
            else:
                x2 = 0.0
                y2 = 0.0
                z2 = 0.0

            # Leg 3 Calculations
            denominator_leg3 = np.sqrt(4 * gamma**2 + alpha**2 + 2 * np.sqrt(3) * alpha * beta + 3 * beta**2)
            if denominator_leg3 != 0:
                x3 = (-L * gamma) / denominator_leg3
                y3 = (-np.sqrt(3) * L * gamma) / denominator_leg3
                z3 = h + ((L * (alpha + np.sqrt(3) * beta)) / denominator_leg3)
            else:
                x3 = 0.0
                y3 = 0.0
                z3 = 0.0

            # Theta Angles Calculations
            L_1 = 0.06  # Bottom arm in meters
            L_2 = 0.0805  # Top arm in meters
            L_m = 0.10206  # Additional length parameter

            # Link 1 Theta Calculations
            L_a1 = np.sqrt(x1**2 + y1**2 + z1**2)
            try:
                angle_inner1 = np.arccos(z1 / L_a1) + 1.5708  # Adding 90 degrees in radians
                acos_arg1 = (-L_2**2 + (x1**2 + y1**2 + z1**2) +
                             2 * L_m * L_a1 * np.cos(angle_inner1) +
                             L_m**2) / (2 * L_1)
                if -1 <= acos_arg1 <= 1:
                    theta_1 = np.degrees(np.arccos(acos_arg1))
                else:
                    theta_1 = 90  # Default or handle error
            except:
                theta_1 = 90  # Default or handle error

            theta_1 = np.clip(theta_1, 90, 179)

            # Link 2 Theta Calculations
            L_a2 = np.sqrt(x2**2 + y2**2 + z2**2)
            try:
                angle_inner2 = np.arccos(z2 / L_a2) + 1.5708
                acos_arg2 = (-L_2**2 + (x2**2 + y2**2 + z2**2) +
                             2 * L_m * L_a2 * np.cos(angle_inner2) +
                             L_m**2) / (2 * L_1)
                if -1 <= acos_arg2 <= 1:
                    theta_2 = np.degrees(np.arccos(acos_arg2))
                else:
                    theta_2 = 90  # Default or handle error
            except:
                theta_2 = 90  # Default or handle error

            theta_2 = np.clip(theta_2, 90, 179)

            # Link 3 Theta Calculations
            L_a3 = np.sqrt(x3**2 + y3**2 + z3**2)
            try:
                angle_inner3 = np.arccos(z3 / L_a3) + 1.5708
                acos_arg3 = (-L_2**2 + (x3**2 + y3**2 + z3**2) +
                             2 * L_m * L_a3 * np.cos(angle_inner3) +
                             L_m**2) / (2 * L_1)
                if -1 <= acos_arg3 <= 1:
                    theta_3 = np.degrees(np.arccos(acos_arg3))
                else:
                    theta_3 = 90  # Default or handle error
            except:
                theta_3 = 90  # Default or handle error

            theta_3 = np.clip(theta_3, 90, 179)

            # Store results
            self.results['V'].append(V)
            self.results['theta'].append(theta_deg)
            self.results['phi'].append(phi_deg)
            self.results['alpha'].append(alpha)
            self.results['beta'].append(beta)
            self.results['gamma'].append(gamma)
            self.results['theta_1'].append(theta_1)
            self.results['theta_2'].append(theta_2)
            self.results['theta_3'].append(theta_3)

            callback(theta_1, theta_2, theta_3)

            print(f"V: {V:.5f} m/s")
            print(f"Theta: {theta_deg:.2f} degrees")
            print(f"Phi: {phi_deg:.2f} degrees")
            print(f"Alpha: {alpha:.5f}")
            print(f"Beta: {beta:.5f}")
            print(f"Gamma: {gamma:.5f}")
            print(f"Theta_1: {theta_1:.2f} degrees")
            print(f"Theta_2: {theta_2:.2f} degrees")
            print(f"Theta_3: {theta_3:.2f} degrees")
            print("-" * 40)

            time.sleep(self.delay)

    def get_results(self):
        """
        Returns the stored results of the path following and PID control.

        Returns:
            dict: A dictionary containing lists of computed values.
        """
        return self.results
