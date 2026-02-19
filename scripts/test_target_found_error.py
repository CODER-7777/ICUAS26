#!/usr/bin/env python3
"""
Test script: subscribes to /target_found and logs position error vs ground truth
for ArUco markers with id in [1, 10] only.

Ground truth from Gazebo world (same as in aruco_mission_node.cpp).
"""

import math
import rclpy
from rclpy.node import Node

try:
    from icuas25_msgs.msg import TargetInfo
except ImportError:
    try:
        from icuas26_msgs.msg import TargetInfo
    except ImportError:
        raise ImportError("Need icuas25_msgs or icuas26_msgs. Install the package that provides TargetInfo.")


# Ground truth (x, y, z) for markers 1-10 from Gazebo world
# Only IDs present in the world in range [1, 10] are included.
GROUND_TRUTH_1_10 = {
    2: (-2.2373, 1.9367, 0.9951),
    4: (0.5277, -4.3226, 3.1164),
    6: (0.4546, -3.8881, 1.0082),
    7: (0.7506, -3.8684, 2.2749),
    8: (5.9808, 5.9191, 1.8864),
    9: (-2.0129, -2.0421, 1.9796),
}


def euclidean_error(ax, ay, az, bx, by, bz):
    return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2 + (az - bz) ** 2)


class TargetFoundErrorLogger(Node):
    def __init__(self):
        super().__init__("target_found_error_logger")
        self.sub = self.create_subscription(
            TargetInfo,
            "/target_found",
            self.callback,
            10,
        )
        self.get_logger().info(
            "Subscribed to /target_found; logging error vs ground truth for marker ids 1-10 only."
        )

    def callback(self, msg):
        marker_id = int(msg.id)
        if marker_id < 1 or marker_id > 10:
            return
        if marker_id not in GROUND_TRUTH_1_10:            return  # no ground truth for this id in 1-10

        gx, gy, gz = GROUND_TRUTH_1_10[marker_id]
        x = msg.location.x
        y = msg.location.y
        z = msg.location.z

        err = euclidean_error(x, y, z, gx, gy, gz)
        err_cm = err * 100.0

        self.get_logger().info(
            f"Marker {marker_id} | detected=({x:.3f}, {y:.3f}, {z:.3f}) "
            f"| ground_truth=({gx:.3f}, {gy:.3f}, {gz:.3f}) "
            f"| error={err:.4f} m ({err_cm:.2f} cm)"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TargetFoundErrorLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
