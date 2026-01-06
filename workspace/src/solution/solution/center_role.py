#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
from functools import partial

from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import BatteryState
from crazyflie_interfaces.srv import Takeoff, GoTo, Land
from builtin_interfaces.msg import Duration
from solution_interfaces.srv import Assign


class centerController(Node):
    def __init__(self):
        super().__init__('tagger_controller')
        self.center_name = None
        self.charging_drone_name = None
        self.search_squad_names = []
        self.takeoff_complete = False
        self.fixed_z = 35.0
        self.fixed_x = 79.0
        self.fixed_y = 79.0
        self.target_z = self.fixed_z
        self.track_duration_ns = 200_000_000
        self.drone_battery_levels = {}
        self.is_swapping_center = False
        self.battery_low_threshold = 90.1
        self.original_center_name = None
        self.replacement_center_name = None

        self.takeoff_clients = {}
        self.goto_clients = {}
        self.land_clients = {}
        self.assign_client = self.create_client(Assign, 'assign_roles')

        self.center_takeoff_client = None
        self.center_goto_client = None
        self.battery_subs = {}

        self.roles_sub = self.create_subscription(String, '/drone_roles', self.roles_callback, 10)

        self.stabilization_timer = None
        self.retry_takeoff_timer = None

        self.get_logger().info("✅ center Controller node started.")
        self.assign_timer = self.create_timer(3.0, self.initial_assign_center)

        # ✅ Periodic logging timer — log AGV + center + Battery every 5s
        self.logging_timer = self.create_timer(5.0, self.periodic_log_callback)

    # ============================================================
    # Periodic Logging
    # ============================================================
    def periodic_log_callback(self):
        """Logs center battery, AGV pose, and center tracking state every 5 seconds."""
        if self.center_name:
            battery = self.drone_battery_levels.get(self.center_name, None)
            if battery is not None:
                self.get_logger().info(f"[Battery] center {self.center_name}: {battery:.1f}%")
            else:
                self.get_logger().info(f"[Battery] center {self.center_name}: No data")
        else:
            self.get_logger().info("[Battery] center not assigned yet.")

    # ============================================================
    # Initial Role Assignment
    # ============================================================
    def initial_assign_center(self):
        if not self.assign_client.service_is_ready():
            self.get_logger().info("Waiting for Assign service...")
            return
        self.get_logger().info("Assigning initial center: cf_2")
        req = Assign.Request()
        req.drone_name = "cf_2"
        req.role = "center"
        future = self.assign_client.call_async(req)
        future.add_done_callback(self.initial_assign_callback)
        self.assign_timer.cancel()

    def initial_assign_callback(self, future):
        try:
            future.result()
            self.get_logger().info("Initial center assignment to cf_2 successful.")
        except Exception as e:
            self.get_logger().error(f"Initial center assignment failed: {e}")

    # ============================================================
    # Roles callback
    # ============================================================
    def roles_callback(self, msg):
        if self.center_name is not None:
            return
        try:
            roles = json.loads(msg.data)
            center_list = roles.get("center")
            charging_drone_name = roles.get("charging")
            search_list = roles.get("search_squad")

            if not center_list or len(center_list) == 0:
                return

            self.center_name = center_list[0]
            self.charging_drone_name = charging_drone_name[0] if charging_drone_name else None
            self.search_squad_names = search_list

            all_managed_drones = [self.center_name] + self.search_squad_names
            if self.charging_drone_name:
                all_managed_drones.append(self.charging_drone_name)

            self.setup_drone_comms(all_managed_drones)
            self.center_takeoff_client = self.takeoff_clients.get(self.center_name)
            self.center_goto_client = self.goto_clients.get(self.center_name)
            self.start_mission_sequence()
        except Exception as e:
            self.get_logger().error(f"Error in roles callback: {e}")

    # ============================================================
    # Drone Communication Setup
    # ============================================================
    def setup_drone_comms(self, drone_names):
        for drone_name in set(drone_names):
            if drone_name is None:
                continue
            self.takeoff_clients[drone_name] = self.create_client(Takeoff, f"/{drone_name}/takeoff")
            self.goto_clients[drone_name] = self.create_client(GoTo, f"/{drone_name}/go_to")
            self.land_clients[drone_name] = self.create_client(Land, f"/{drone_name}/land")

            topic = f"/{drone_name}/battery_status"
            callback = partial(self.battery_callback, drone_name=drone_name)
            self.battery_subs[drone_name] = self.create_subscription(BatteryState, topic, callback, 10)
            self.drone_battery_levels[drone_name] = 100.0

    # ============================================================
    # Mission Sequence
    # ============================================================
    def start_mission_sequence(self):
        if self.center_takeoff_client is None or self.center_goto_client is None:
            self.get_logger().error(f"center clients for {self.center_name} not initialized.")
            return
        self.wait_for_services_timer = self.create_timer(1.0, self.wait_for_services_callback)

    def wait_for_services_callback(self):
        takeoff_ready = self.center_takeoff_client.service_is_ready()
        goto_ready = self.center_goto_client.service_is_ready()
        if not takeoff_ready or not goto_ready:
            return
        self.get_logger().info(f"center {self.center_name} services are ready.")
        self.wait_for_services_timer.cancel()
        self.call_takeoff_service()

    def finish_takeoff_wait_callback(self):
        if self.stabilization_timer:
            self.stabilization_timer.cancel()
            self.stabilization_timer = None
        self.get_logger().info("15s stabilization wait complete. Takeoff is now marked as complete.")
        self.takeoff_complete = True
        self.go_to_center_callback()

    def call_takeoff_service(self):
        self.get_logger().info(f"Requesting takeoff for {self.center_name} to {self.fixed_z}m")
        request = Takeoff.Request()
        request.height = self.fixed_z
        request.duration = Duration(sec=2, nanosec=0)
        request.group_mask = 0
        future = self.center_takeoff_client.call_async(request)
        future.add_done_callback(self.takeoff_response_callback)

    def takeoff_response_callback(self, future):
        try:
            future.result()
            self.get_logger().info(f"Takeoff service successful for {self.center_name}")
            if self.retry_takeoff_timer:
                self.retry_takeoff_timer.cancel()
                self.retry_takeoff_timer = None
            self.stabilization_timer = self.create_timer(15.0, self.finish_takeoff_wait_callback)
        except Exception as e:
            self.get_logger().error(f"Takeoff service call failed: {e}")
            if self.retry_takeoff_timer:
                self.retry_takeoff_timer.cancel()
            self.retry_takeoff_timer = self.create_timer(5.0, self.retry_takeoff_callback)

    def retry_takeoff_callback(self):
        if self.retry_takeoff_timer:
            self.retry_takeoff_timer.cancel()
            self.retry_takeoff_timer = None
        self.get_logger().info("Retrying takeoff...")
        self.call_takeoff_service()

    # ============================================================
    # Going to Center
    # ============================================================
    def go_to_center_callback(self):
        if not self.takeoff_complete or self.is_swapping_center:
            return
        if self.center_goto_client is None or not self.center_goto_client.service_is_ready():
            return
        request = GoTo.Request()
        request.group_mask = 0
        request.relative = False
        request.goal.x = self.fixed_x
        request.goal.y = self.fixed_y
        request.goal.z = self.fixed_z
        request.yaw = 0.0
        request.duration = Duration(sec=0, nanosec=self.track_duration_ns)
        self.center_goto_client.call_async(request)
        self.get_logger().info(f"Went to center")

    # ============================================================
    # Battery Monitoring & Swap Logic
    # ============================================================
    def battery_callback(self, msg, drone_name):
        percentage = msg.percentage
        self.drone_battery_levels[drone_name] = percentage
        if drone_name == self.center_name and percentage < self.battery_low_threshold and not self.is_swapping_center:
            self.initiate_center_swap()

    def initiate_center_swap(self):
        self.is_swapping_center = True
        best_drone = None
        max_battery = -1.0
        active_search_squad = [name for name in self.search_squad_names if name != self.center_name]
        for drone_name in active_search_squad:
            battery = self.drone_battery_levels.get(drone_name, 0.0)
            if battery > max_battery:
                max_battery = battery
                best_drone = drone_name
        if best_drone is None or max_battery < self.battery_low_threshold:
            self.is_swapping_center = False
            return
        self.original_center_name = self.center_name
        self.replacement_center_name = best_drone
        takeoff_cli = self.takeoff_clients.get(self.replacement_center_name)
        goto_cli = self.goto_clients.get(self.replacement_center_name)
        if takeoff_cli is None or not takeoff_cli.service_is_ready() or goto_cli is None or not goto_cli.service_is_ready():
            self.is_swapping_center = False
            return
        if self.target_z == self.fixed_z:
            self.target_z = self.fixed_z + 5.0
        else:
            self.target_z = self.fixed_z
        req = Takeoff.Request()
        req.height = self.target_z
        req.duration = Duration(sec=3, nanosec=0)
        req.group_mask = 0
        future = takeoff_cli.call_async(req)
        future.add_done_callback(self.replacement_takeoff_callback)

    def replacement_takeoff_callback(self, future):
        try:
            future.result()
        except Exception:
            self.is_swapping_center = False
            return
        target_pos = Point()
        target_pos.x = self.fixed_x
        target_pos.y = self.fixed_y
        target_pos.z = self.target_z
        req = GoTo.Request()
        req.goal = target_pos
        req.relative = False
        req.duration = Duration(sec=5, nanosec=0)
        goto_cli = self.goto_clients.get(self.replacement_center_name)
        future = goto_cli.call_async(req)
        future.add_done_callback(self.replacement_goto_callback)

    def replacement_goto_callback(self, future):
        try:
            future.result()
        except Exception:
            self.is_swapping_center = False
            return
        target_pos = Point(x=0.0, y=0.0, z=self.fixed_z)
        goto_cli = self.goto_clients.get(self.original_center_name)
        if goto_cli is None or not goto_cli.service_is_ready():
            self.is_swapping_center = False
            return
        req = GoTo.Request()
        req.goal = target_pos
        req.relative = False
        req.duration = Duration(sec=5, nanosec=0)
        future = goto_cli.call_async(req)
        future.add_done_callback(self.swap_step_4_land_old_center)

    def swap_step_4_land_old_center(self, future):
        try:
            future.result()
        except Exception:
            self.is_swapping_center = False
            return
        land_cli = self.land_clients.get(self.original_center_name)
        if land_cli is None or not land_cli.service_is_ready():
            self.is_swapping_center = False
            return
        req = Land.Request()
        req.height = 0.0
        req.duration = Duration(sec=3, nanosec=0)
        req.group_mask = 0
        future = land_cli.call_async(req)
        future.add_done_callback(self.swap_step_5_assign_new_center)

    def swap_step_5_assign_new_center(self, future):
        try:
            future.result()
        except Exception:
            self.is_swapping_center = False
            return
        req1 = Assign.Request()
        req1.drone_name = self.replacement_center_name
        req1.role = "center"
        future1 = self.assign_client.call_async(req1)
        future1.add_done_callback(self.swap_step_6_assign_old_center)

    def swap_step_6_assign_old_center(self, future):
        try:
            future.result()
        except Exception:
            self.is_swapping_center = False
            return
        req2 = Assign.Request()
        req2.drone_name = self.original_center_name
        req2.role = "charging"
        future2 = self.assign_client.call_async(req2)
        future2.add_done_callback(self.swap_step_7_finalize_swap)

    def swap_step_7_finalize_swap(self, future):
        try:
            future.result()
        except Exception:
            self.is_swapping_center = False
            return
        self.center_name = self.replacement_center_name
        self.charging_drone_name = self.original_center_name
        if self.center_name in self.search_squad_names:
            self.search_squad_names.remove(self.center_name)
        if self.original_center_name not in self.search_squad_names:
            self.search_squad_names.append(self.original_center_name)
        self.center_takeoff_client = self.takeoff_clients.get(self.center_name)
        self.center_goto_client = self.goto_clients.get(self.center_name)
        self.replacement_center_name = None
        self.original_center_name = None
        self.is_swapping_center = False


def main(args=None):
    rclpy.init(args=args)
    node = centerController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
