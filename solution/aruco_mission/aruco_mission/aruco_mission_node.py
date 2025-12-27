#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import cv2.aruco as aruco
import numpy as np
import math
from tf_transformations import euler_from_quaternion

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from icuas25_msgs.msg import TargetInfo
from rclpy.qos import qos_profile_sensor_data


# ============================================================
# PER-DRONE CONTEXT
# ============================================================

class DroneContext:
    def __init__(self, cf):
        self.cf = cf
        self.pose = None
        self.K = None
        self.D = None
        self.marker_seen = set()   # prevent spamming same marker


# ============================================================
# MULTI-DRONE ARUCO DETECTOR
# ============================================================

class MultiArucoDetector(Node):

    def __init__(self):
        super().__init__('multi_aruco_detector')

        # ---------------- CONFIG ----------------
        self.cf_ids = ['cf_1', 'cf_2', 'cf_3', 'cf_4', 'cf_5']
        self.marker_size = 0.25  # meters

        # ---------------- ARUCO ----------------
        self.bridge = CvBridge()
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
        # self.aruco_params = aruco.DetectorParameters()
        self.aruco_params = aruco.DetectorParameters_create()

        # ---------------- GLOBAL TARGET PUB ----------------
        self.global_target_pub = self.create_publisher(
            TargetInfo,
            '/target_node',
            10
        )

        # ---------------- DRONES ----------------
        self.drones = {}

        for cf in self.cf_ids:
            ctx = DroneContext(cf)
            self.drones[cf] = ctx

            # Subscribers
            self.create_subscription(
                Image, f'/{cf}/image',
                lambda m, c=cf: self.image_cb(m, c),
                qos_profile_sensor_data
            )

            self.create_subscription(
                CameraInfo, f'/{cf}/camera_info',
                lambda m, c=cf: self.camera_info_cb(m, c),
                10
            )

            self.create_subscription(
                PoseStamped, f'/{cf}/pose',
                lambda m, c=cf: self.pose_cb(m, c),
                10
            )

            # Per-drone target publisher
            ctx.target_pub = self.create_publisher(
                TargetInfo,
                f'/{cf}/target_found',
                10
            )

        self.get_logger().info("✅ Multi-Drone ArUco Detector Started (Perception-Only)")

    # ======================================================
    # CALLBACKS
    # ======================================================

    def camera_info_cb(self, msg, cf):
        ctx = self.drones[cf]
        if ctx.K is None:
            ctx.K = np.array(msg.k).reshape(3, 3)
            ctx.D = np.array(msg.d)

    def pose_cb(self, msg, cf):
        self.drones[cf].pose = msg

    def image_cb(self, msg, cf):
        ctx = self.drones[cf]
        if ctx.pose is None or ctx.K is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )

        if ids is None:
            cv2.imshow(cf, frame)
            cv2.waitKey(1)
            return

        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners, self.marker_size, ctx.K, ctx.D
        )

        aruco.drawDetectedMarkers(frame, corners, ids)

        for i, marker_id in enumerate(ids.flatten()):
            if marker_id in ctx.marker_seen:
                continue

            ctx.marker_seen.add(marker_id)
            self.publish_target(cf, marker_id, tvecs[i][0])

        cv2.imshow(cf, frame)
        cv2.waitKey(1)

    # ======================================================
    # TARGET PUBLISH
    # ======================================================

    def publish_target(self, cf, marker_id, tvec):
        ctx = self.drones[cf]
        pos = ctx.pose.pose.position
        q = ctx.pose.pose.orientation

        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # Camera → Drone frame
        cam_to_drone = np.array([
            [ 0,  0,  1 ],
            [-1,  0,  0 ],
            [ 0, -1,  0 ]
        ])

        # Drone → World (yaw only)
        R = np.array([
            [ math.cos(yaw), -math.sin(yaw), 0 ],
            [ math.sin(yaw),  math.cos(yaw), 0 ],
            [ 0,              0,             1 ]
        ])

        marker_drone = cam_to_drone @ tvec
        marker_world = R @ marker_drone

        world_x = pos.x + marker_world[0]
        world_y = pos.y + marker_world[1]
        world_z = pos.z + marker_world[2]

        msg = TargetInfo()
        msg.id = int(marker_id)
        msg.location.x = float(world_x)
        msg.location.y = float(world_y)
        msg.location.z = float(world_z)

        # Publish per-drone
        ctx.target_pub.publish(msg)

        # Publish global
        self.global_target_pub.publish(msg)

        self.get_logger().info(
            f"[{cf}] 🎯 Marker {marker_id} @ "
            f"({world_x:.2f}, {world_y:.2f}, {world_z:.2f})"
        )


# ======================================================
# MAIN
# ======================================================

def main():
    rclpy.init()
    node = MultiArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
