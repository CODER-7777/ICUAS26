import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from collections import deque
import matplotlib.pyplot as plt

class ContinuousAccelerationPredictor(Node):
    def __init__(self):
        super().__init__('continuous_accel_predictor')
        
        self.subscription = self.create_subscription(
            Point, '/AGV/pose', self.pose_callback, 10)
        
        # Buffer for the 1.0s sliding physics window
        self.history = deque()
        self.window_size = 1.0 
        
        # Buffer for the 3.0s prediction-to-actual alignment
        self.prediction_queue = deque()
        
        # Storage for continuous plotting
        self.timestamps = []
        self.actual_x, self.actual_y = [], []
        self.predicted_x, self.predicted_y = [], []
        
        self.start_time = None
        self.prediction_horizon = 3.0 # The "Shift"
        
        self.get_logger().info("--- Continuous 3s Predictor Started ---")
        self.get_logger().info("Plotting continuous prediction line aligned with truth.")

    def pose_callback(self, msg):
        curr_time = self.get_clock().now().nanoseconds / 1e9
        if self.start_time is None:
            self.start_time = curr_time
        
        elapsed = curr_time - self.start_time
        
        # Stop and plot after 120 seconds
        if elapsed > 120.0:
            self.plot_results()
            return

        # 1. Alignment Logic: Compare current pose with prediction made 3s ago
        # If we have a prediction in the queue that matches this current timestamp
        while self.prediction_queue and curr_time >= self.prediction_queue[0][0]:
            target_t, px, py = self.prediction_queue.popleft()
            
            # If the prediction's target time matches current time (within 0.05s tolerance)
            if abs(curr_time - target_t) < 0.05:
                self.timestamps.append(elapsed)
                self.actual_x.append(msg.x)
                self.actual_y.append(msg.y)
                self.predicted_x.append(px)
                self.predicted_y.append(py)

        # 2. Physics logic: Update the 1.0s window (e.g., 0.0 to 0.9, 0.1 to 1.0)
        self.history.append((curr_time, msg.x, msg.y))
        while self.history and (curr_time - self.history[0][0] > self.window_size):
            self.history.popleft()

        # 3. Calculation logic: Use the current 1.0s window to predict 3s into the future
        if len(self.history) > 3:
            pred_x, pred_y = self.calculate_3s_physics()
            # Store the prediction with its target arrival time
            self.prediction_queue.append((curr_time + self.prediction_horizon, pred_x, pred_y))

    def calculate_3s_physics(self):
        # Physics points: t0/x0/y0 is start of 1s window, tn/xn/yn is now
        t0, x0, y0 = self.history[0]
        t1, x1, y1 = self.history[1]
        tn_1, xn_1, yn_1 = self.history[-2]
        tn, xn, yn = self.history[-1]
        
        dt_start, dt_end, dt_total = (t1 - t0), (tn - tn_1), (tn - t0)
        dt_p = self.prediction_horizon

        # Current Velocities (Instantaneous)
        vx_curr = (xn - xn_1) / dt_end
        vy_curr = (yn - yn_1) / dt_end
        
        # Accelerations (Average over the 1s window)
        ax = (vx_curr - ((x1 - x0) / dt_start)) / dt_total
        ay = (vy_curr - ((y1 - y0) / dt_start)) / dt_total

        # Predict position at t_now + 3.0s
        px = xn + (vx_curr * dt_p) + (0.5 * ax * (dt_p**2))
        py = yn + (vy_curr * dt_p) + (0.5 * ay * (dt_p**2))
        
        return px, py