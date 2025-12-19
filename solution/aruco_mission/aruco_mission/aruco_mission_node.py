#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import cv2
import cv2.aruco as aruco
import numpy as np
from tf_transformations import euler_from_quaternion

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Point
from cv_bridge import CvBridge
from icuas25_msgs.msg import TargetInfo
from rclpy.qos import qos_profile_sensor_data

from crazyflie_interfaces.srv import Arm, Takeoff, GoTo, Land
from builtin_interfaces.msg import Duration

class ArucoMission(Node):

    def __init__(self):
        super().__init__('aruco_mission_node')

        # ================= CONFIG =================
        self.waypoints = [
            Point(x=-1.73, y=-4.94, z=2.16),
            Point(x=-2.36, y=-2.29, z=1.98),
        ]

        self.TAKEOFF_Z = self.waypoints[0].z
        self.HOVER_TIME = 5.0       # Duration for all intermediate hovers
        self.GOTO_DURATION = 20.0    # Very slow transit for 46% RTF stability
        self.SPIN_STEP = math.pi / 4
        self.YAW_EPS = math.radians(7.0)
        self.DIST_THRESHOLD = 0.1

        
        # ================= STATE =================
        self.state = "IDLE"
        self.current_wp = 0
        self.command_sent = False
        self.marker_found = False
        self.target_yaw = 0.0
        self.scan_accum = 0.0
        self.hover_start = None

        # ================= PERCEPTION =================
        self.K = None
        self.D = None
        self.pose = None
        self.bridge = CvBridge()

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
        try:
            self.aruco_params = aruco.DetectorParameters()
        except AttributeError:
            self.aruco_params = aruco.DetectorParameters_create()
        self.marker_size = 0.25

        # ================= ROS =================
        self.create_subscription(Image, '/cf_1/image', self.image_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/cf_1/camera_info', self.camera_info_cb, 10)
        self.create_subscription(PoseStamped, '/cf_1/pose', self.pose_cb, 10)

        self.target_pub = self.create_publisher(TargetInfo, '/cf_1/target_found', 10)

        self.arm_cli = self.create_client(Arm, '/cf_1/arm')
        self.takeoff_cli = self.create_client(Takeoff, '/cf_1/takeoff')
        self.goto_cli = self.create_client(GoTo, '/cf_1/go_to')
        self.land_cli = self.create_client(Land, '/cf_1/land')

        self.control_timer = self.create_timer(1.0, self.control_loop)

        cv2.namedWindow("Camera View", cv2.WINDOW_NORMAL)
        self.get_logger().info("✅ Mission Node Started - Triple Hover Mode Active")

    # ======================================================
    # CALLBACKS
    # ======================================================

    def camera_info_cb(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.D = np.array(msg.d)

    def pose_cb(self, msg):
        self.pose = msg

    def image_cb(self, msg):
        if self.pose is None or self.K is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.state == "SPIN_SEARCH" and not self.marker_found:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )

            if ids is not None:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.K, self.D
                )

                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                # Process ALL detected markers (safe)
                for i, marker_id in enumerate(ids.flatten()):
                    self.handle_detection(
                        marker_id,
                        rvecs[i][0],
                        tvecs[i][0]
                    )
                    break  # only first marker (competition safe)

        cv2.imshow("Camera View", frame)
        cv2.waitKey(1)


    # ======================================================
    # FSM
    # ======================================================

    def control_loop(self):
        if self.pose is None: return
        pos = self.pose.pose.position

        if self.state == "IDLE":
            self.arm_and_takeoff()

        elif self.state == "TAKEOFF_HOVER":
            # Wait for altitude before proceeding
            if abs(pos.z - self.TAKEOFF_Z) < self.DIST_THRESHOLD:
                self.transition("ALTITUDE_ALIGN")

        elif self.state == "ALTITUDE_ALIGN":
            target_z = self.waypoints[self.current_wp].z
            if not self.command_sent:
                self.goto_altitude(target_z)
            elif abs(pos.z - target_z) < self.DIST_THRESHOLD:
                # Insert hover between Alt-Align and XY-Move
                self.transition("HOVER_AFTER_ALT")

        elif self.state == "HOVER_AFTER_ALT":
            self.hover_then("XY_MOVE")

        elif self.state == "XY_MOVE":
            wp = self.waypoints[self.current_wp]
            if not self.command_sent:
                self.goto_xy(wp)
            elif self.get_dist(pos, wp, ignore_z=True) < self.DIST_THRESHOLD:
                self.transition("HOVER_AT_WP")

        elif self.state == "HOVER_AT_WP":
            self.hover_then("SPIN_SEARCH")

        elif self.state == "SPIN_SEARCH":
            self.spin_search()

        elif self.state == "HOVER_AFTER_DETECTION":
            self.hover_then("ADVANCE_WP")
            
        elif self.state == "ADVANCE_WP":
            self.advance_waypoint()
            
        elif self.state == "LAND":
            self.land()

    # ======================================================
    # STATE ACTIONS
    # ======================================================

    def arm_and_takeoff(self):
        if not self.arm_cli.service_is_ready(): return
        self.arm_cli.call_async(Arm.Request(arm=True))
        req = Takeoff.Request()
        req.height = self.TAKEOFF_Z
        req.duration = Duration(sec=10)
        self.takeoff_cli.call_async(req)
        self.transition("TAKEOFF_HOVER")

    def goto_altitude(self, z):
        pos = self.pose.pose.position
        p = Point(x=pos.x, y=pos.y, z=z)
        
        # Use a much shorter duration for vertical alignment
        # 10 seconds is usually smooth for 1-2 meters
        self.send_goto(p, duration_override=20)

    def goto_xy(self, wp):
        pos = self.pose.pose.position
        p = Point(x=wp.x, y=wp.y, z=pos.z)
        self.send_goto(p)

    def spin_search(self):

        if self.marker_found:
            return
        yaw = self.get_yaw()
        err = self.angle_diff(self.target_yaw, yaw)

        if abs(err) < self.YAW_EPS:
            if self.scan_accum > 2 * math.pi:
                self.advance_waypoint()
                return
            self.target_yaw += self.SPIN_STEP
            self.scan_accum += abs(self.SPIN_STEP)
            self.command_sent = False

        if not self.command_sent:
            self.send_goto(self.pose.pose.position, self.normalize(self.target_yaw))

    def hover_then(self, next_state):
        if self.hover_start is None:
            pos = self.pose.pose.position
            yaw = self.get_yaw()

            hold = Point(x=pos.x, y=pos.y, z=pos.z)
            self.send_goto(hold, yaw=yaw, duration_override=int(self.HOVER_TIME))

            self.hover_start = self.get_clock().now()
            self.get_logger().info(f"🛑 HOLDING for {self.HOVER_TIME}s")

        elapsed = (self.get_clock().now() - self.hover_start).nanoseconds * 1e-9
        if elapsed > self.HOVER_TIME:
            self.hover_start = None
            self.transition(next_state)


    def send_goto(self, point, yaw=None, duration_override=None):
        if not self.goto_cli.service_is_ready(): return
        
        # Use override if provided, else use global default
        duration = duration_override if duration_override is not None else self.GOTO_DURATION
        
        req = GoTo.Request()
        req.goal = point
        req.relative = False
        req.duration = Duration(sec=int(duration))
        if yaw is not None:
            req.yaw = float(yaw)
            
        self.goto_cli.call_async(req)
        self.command_sent = True

    def get_dist(self, p1, p2, ignore_z=False):
        dz = 0 if ignore_z else (p1.z - p2.z)
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + dz**2)

    def advance_waypoint(self):
        self.current_wp += 1
        self.command_sent = False
        self.marker_found = False
        self.scan_accum = 0.0
        self.hover_start = None # Reset hover timer for next transitions
        
        if self.current_wp >= len(self.waypoints):
            self.transition("LAND")
        else:
            self.get_logger().info(f"Advancing to WP {self.current_wp + 1}")
            self.transition("ALTITUDE_ALIGN")

    def handle_detection(self, marker_id, rvec, tvec):
        if self.marker_found:
            return

        self.marker_found = True

        # ----------------------------
        # 1. Marker pose in CAMERA frame
        # ----------------------------
        marker_cam = np.array([
            tvec[0],
            tvec[1],
            tvec[2],
            1.0
        ])

        # ----------------------------
        # 2. Drone pose in WORLD frame
        # ----------------------------
        pos = self.pose.pose.position
        q = self.pose.pose.orientation

        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # Rotation matrix (world <- drone)
        R = np.array([
            [ math.cos(yaw), -math.sin(yaw), 0 ],
            [ math.sin(yaw),  math.cos(yaw), 0 ],
            [ 0,              0,             1 ]
        ])

        # Camera frame assumption:
        # X_cam → right, Y_cam → down, Z_cam → forward
        # Drone frame:
        # X → forward, Y → left, Z → up
        cam_to_drone = np.array([
            [ 0,  0,  1 ],
            [-1,  0,  0 ],
            [ 0, -1,  0 ]
        ])

        # ----------------------------
        # 3. Transform to WORLD frame
        # ----------------------------
        marker_drone = cam_to_drone @ marker_cam[:3]
        marker_world = R @ marker_drone

        world_x = pos.x + marker_world[0]
        world_y = pos.y + marker_world[1]
        world_z = pos.z + marker_world[2]

        # ----------------------------
        # 4. Publish TargetInfo
        # ----------------------------
        msg = TargetInfo()
        msg.id = int(marker_id)
        msg.location.x = float(world_x)
        msg.location.y = float(world_y)
        msg.location.z = float(world_z)

        self.target_pub.publish(msg)

        self.get_logger().info(
            f"🎯 Marker {marker_id} WORLD @ "
            f"({world_x:.2f}, {world_y:.2f}, {world_z:.2f})"
        )

        # HARD HOLD (stop spiral)
        hold = Point(x=pos.x, y=pos.y, z=pos.z)
        self.send_goto(hold, yaw=yaw, duration_override=5)

        self.transition("HOVER_AFTER_DETECTION")


    def land(self):
        if not self.land_cli.service_is_ready(): return
        req = Land.Request()
        req.height = 0.0
        req.duration = Duration(sec=6)
        self.land_cli.call_async(req)

    def transition(self, new_state):
        self.command_sent = False
        self.state = new_state
        self.get_logger().info(f"➡ STATE → {new_state}")

    def get_yaw(self):
        return euler_from_quaternion([self.pose.pose.orientation.x, self.pose.pose.orientation.y, 
                                      self.pose.pose.orientation.z, self.pose.pose.orientation.w])[2]

    def angle_diff(self, a, b):
        return (a - b + math.pi) % (2*math.pi) - math.pi

    def normalize(self, a):
        return math.atan2(math.sin(a), math.cos(a))

    def on_shutdown(self):
        self.get_logger().warn("🛑 Shutdown detected — landing & disarming")

        # LAND
        if self.land_cli.service_is_ready():
            land = Land.Request()
            land.height = 0.0
            land.duration = Duration(sec=3)
            self.land_cli.call_async(land)

        # DISARM
        if self.arm_cli.service_is_ready():
            self.arm_cli.call_async(Arm.Request(arm=False))


def main():
    rclpy.init()
    node = ArucoMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()