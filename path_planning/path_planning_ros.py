#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import csv
import time

from crazyflie_interfaces.srv import GoTo, Takeoff, Land
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        # Service clients
        self.takeoff_client = self.create_client(Takeoff, '/cf_1/takeoff')
        self.goto_client = self.create_client(GoTo, '/cf_1/go_to')
        self.land_client = self.create_client(Land, '/cf_1/land')

        self.fixed_z = 5.0  # Takeoff height
        self.get_logger().info("Waiting for Crazyflie services...")
        self.takeoff_client.wait_for_service()
        self.goto_client.wait_for_service()
        self.land_client.wait_for_service()
        self.get_logger().info("✅ Connected to Crazyflie services")

        # Load path from CSV
        self.path = self.load_path('final_path_coordinates.csv')
        self.current_index = 0

        # Start sequence
        self.call_takeoff_service()

    def load_path(self, filename):
        """Load (x, y, z) coordinates from file."""
        path = []
        try:
            with open(filename, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    path.append((float(row['x']), float(row['y']), float(row['z'])))
        except Exception as e:
            self.get_logger().error(f"Error reading {filename}: {e}")
        self.get_logger().info(f"Loaded {len(path)} waypoints.")
        return path

    # === TAKEOFF ===
    def call_takeoff_service(self):
        """Take off asynchronously before flying the path."""
        self.get_logger().info(f"🛫 Requesting takeoff to {self.fixed_z} m...")
        request = Takeoff.Request()
        request.height = self.fixed_z
        request.duration = Duration(sec=30, nanosec=0)
        request.group_mask = 0

        future = self.takeoff_client.call_async(request)
        future.add_done_callback(self.takeoff_response_callback)

    def takeoff_response_callback(self, future):
        """Handle takeoff response and start waypoint traversal."""
        try:
            future.result()
            self.get_logger().info("✅ Takeoff complete.")
        except Exception as e:
            self.get_logger().error(f"Takeoff failed: {e}")
            return
        # Wait a bit for drone to stabilize
        time.sleep(2.0)
        self.send_next_waypoint()

    # === GOTO ===
    def send_next_waypoint(self):
        """Send next waypoint in sequence to /cf_1/go_to."""
        if self.current_index >= len(self.path):
            self.get_logger().info("✅ All waypoints visited. Preparing to land.")
            self.call_land_service()
            return

        x, y, z = self.path[self.current_index]
        self.get_logger().info(f"→ Sending waypoint {self.current_index+1}/{len(self.path)}: ({x:.2f}, {y:.2f}, {z:.2f})")

        request = GoTo.Request()
        request.group_mask = 0
        request.relative = False
        request.goal = Point(x=x, y=y, z=z)
        request.yaw = 0.0
        request.duration = Duration(sec=30, nanosec=0)

        future = self.goto_client.call_async(request)
        future.add_done_callback(self.goto_response_callback)

    def goto_response_callback(self, future):
        """After reaching a waypoint, move to the next one."""
        try:
            future.result()
            self.get_logger().info(f"✅ Waypoint {self.current_index+1} complete.")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Failed to reach waypoint {self.current_index+1}: {e}")

        self.current_index += 1
        time.sleep(30.0)
        self.send_next_waypoint()

    # === LAND ===
    def call_land_service(self):
        """Land after completing all waypoints."""
        self.get_logger().info("🛬 Requesting landing...")
        request = Land.Request()
        request.height = 0.0
        request.duration = Duration(sec=5, nanosec=0)
        request.group_mask = 0

        future = self.land_client.call_async(request)
        future.add_done_callback(self.land_response_callback)

    def land_response_callback(self, future):
        try:
            future.result()
            self.get_logger().info("✅ Landed successfully.")
        except Exception as e:
            self.get_logger().error(f"Landing failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
