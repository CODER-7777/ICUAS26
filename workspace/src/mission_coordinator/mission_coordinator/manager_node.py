import rclpy
from rclpy.node import Node
from enum import IntEnum
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Int32

# ===================== CONSTANTS =====================
BATTERY_PERCENTAGE_THRESH = 60.0
CRITICAL_BATTERY_THRESH = 30.0
MAX_CHARGING = 2
NUM_DRONES = 5

# ===================== ROLES =====================
class Role(IntEnum):
    UNASSIGNED = 0
    CHARGE = 1
    SCOUT = 2
    LAND = 3
    FOLLOW = 4
    CENTER = 5

# ===================== DRONE MODEL =====================
class Drone:
    def __init__(self, drone_id):
        self.id = drone_id
        self.batt_percentage = 100.0
        self.role = Role.UNASSIGNED
        self.has_charged = False

# ===================== FLEET MANAGER =====================
class FleetManager(Node):
    def __init__(self):
        super().__init__('fleet_manager')

        # -------------------------
        # Create fleet
        # -------------------------
        self.fleet = [Drone(i+1) for i in range(5)]

        # Publishers
        self.role_pubs = {
            d.id: self.create_publisher(Int32, f'/cf_{d.id}/role', 10)
            for d in self.fleet
        }

        # Subscribers
        for drone in self.fleet:
            self.create_subscription(
                BatteryState,
                f'/cf_{drone.id}/battery_status',
                lambda msg, d=drone: self.battery_callback(msg, d),
                10
            )

        self.assign_initial_roles()

        self.create_timer(1.0, self.manage_fleet)
        self.create_timer(1.0, self.publish_all_roles)
        self.create_timer(2.0, self.print_fleet_status)

        self.get_logger().info("Fleet Manager started")

    # ===================== CALLBACKS =====================
    def battery_callback(self, msg: BatteryState, drone: Drone):
        drone.batt_percentage = msg.percentage

    # ===================== CORE LOGIC =====================
    def manage_fleet(self):
        charging = [d for d in self.fleet if d.role == Role.CHARGE]

        # 1. Finish charging → SCOUT
        for d in charging:
            if d.batt_percentage >= 89.0:
                d.role = Role.SCOUT
                d.has_charged = True
                self.get_logger().info(f'Drone {d.id} charged → SCOUT')


        
        charging = [d for d in self.fleet if d.role == Role.CHARGE]

        while len(charging) > 1:
            d = max(charging, key=lambda d: d.batt_percentage)

            if d.batt_percentage >= 70.0:
                d.role = Role.SCOUT
                d.has_charged = True
                self.get_logger().info(f'Drone {d.id} charged → SCOUT')
            else:
                # No drone can be promoted anymore → exit loop
                break

            # recompute charging list
            charging = [d for d in self.fleet if d.role == Role.CHARGE]

    
        # 2. Land drones that already charged once and now <30%
        for d in self.fleet:
            if d.has_charged and d.batt_percentage < CRITICAL_BATTERY_THRESH:
                if d.role != Role.LAND:
                    d.role = Role.LAND
                    self.get_logger().warn(
                        f'Drone {d.id} LANDED (battery critical, already charged)'
                    )

        landed = [d for d in self.fleet if d.role == Role.LAND]
        if len(landed) == (NUM_DRONES - 3):
            charging = [d for d in self.fleet if d.role == Role.CHARGE]

            while len(charging) > 0:
                d = max(charging, key=lambda d: d.batt_percentage)

                if d.batt_percentage >= 30.0:
                    d.role = Role.SCOUT
                    d.has_charged = True
                    self.get_logger().info(f'Drone {d.id} charged → SCOUT')
                else:
                    # No drone can be promoted anymore → exit loop
                    break

                # recompute charging list
                charging = [d for d in self.fleet if d.role == Role.CHARGE]

                

        # 3. Replace CENTER / FOLLOW if battery <60%
        for role in [Role.CENTER, Role.FOLLOW]:
            active = self.get_drone_by_role(role)
            if active and active.batt_percentage < CRITICAL_BATTERY_THRESH:
                self.replace_active(active, role)

        # 4. Send drones to CHARGE (max 2, only once)
        charging = [d for d in self.fleet if d.role == Role.CHARGE]
        for d in self.fleet:
            if (
                d.batt_percentage < BATTERY_PERCENTAGE_THRESH
                and d.role not in [Role.CHARGE, Role.LAND]
                and not d.has_charged
                and len(charging) < MAX_CHARGING
            ):
                d.role = Role.CHARGE
                charging.append(d)
                self.get_logger().info(f'Drone {d.id} → CHARGE')
            
            if(d.batt_percentage < CRITICAL_BATTERY_THRESH 
                and  len(charging) == MAX_CHARGING
                and d.role not in [Role.CHARGE, Role.LAND]
                and not d.has_charged):
                d.role=Role.CHARGE
                charging.append(d)
                self.get_logger().info(f'Drone {d.id} → CHARGE')


        # 5. Enforce CENTER–FOLLOW pairing
        self.enforce_center_follow_pair()

    # ===================== ROLE MANAGEMENT =====================
    def replace_active(self, failing: Drone, role: Role):
        candidates = [
            d for d in self.fleet
            if d.role == Role.SCOUT and d.batt_percentage > CRITICAL_BATTERY_THRESH
        ]

        if not candidates:
            self.get_logger().error(f'NO replacement available for {role.name}\n Mission Finished')
            return

        replacement = max(candidates, key=lambda d: d.batt_percentage)

        self.get_logger().info(
            f'SWAP: Drone {replacement.id} replaces {failing.id} as {role.name}'
        )

        # failing.role = Role.CHARGE if not failing.has_charged else Role.SCOUT
        failing.role=Role.CHARGE
        replacement.role = role

    def enforce_center_follow_pair(self):
        center = self.get_drone_by_role(Role.CENTER)
        follow = self.get_drone_by_role(Role.FOLLOW)

        if center and follow:
            return

        scouts = sorted(
            [d for d in self.fleet if d.role == Role.SCOUT],
            key=lambda d: d.batt_percentage,
            reverse=True
        )

        if len(scouts) < 2:
            scouts1 = sorted(
                [d for d in self.fleet if d.role == Role.CHARGE],
                    key=lambda d: d.batt_percentage,
                    reverse=True
            )
            scouts.extend(scouts1)
            if len(scouts)<2:
                if(len(scouts)==1):
                    if(center!=None):
                        scouts[0].role=Role.FOLLOW
                    elif(follow!=None):
                        scouts[0].role=Role.CENTER
                    else:
                        self.get_logger().error("Cannot restore CENTER–FOLLOW pair")
                else:
                    self.get_logger().error("Cannot restore CENTER–FOLLOW pair")
                return

        if(center!=None):
            scouts[0].role=Role.FOLLOW
        elif(follow!=None):
            scouts[0].role=Role.CENTER
        else:
            scouts[0].role = Role.CENTER
            scouts[1].role = Role.FOLLOW
        

        self.get_logger().info(
            f'PAIR RESTORED → CENTER: {scouts[0].id}, FOLLOW: {scouts[1].id}'
        )

    def get_drone_by_role(self, role: Role):
        for d in self.fleet:
            if d.role == role:
                return d
        return None

    # ===================== PUBLISHING =====================
    def publish_all_roles(self):
        for d in self.fleet:
            msg = Int32()
            msg.data = int(d.role)
            self.role_pubs[d.id].publish(msg)

    # ===================== STATUS PRINT =====================
    def print_fleet_status(self):
        self.get_logger().info("-" * 40)
        for d in self.fleet:
            self.get_logger().info(
                f"Drone {d.id} | Role: {d.role.name:7} | "
                f"Battery: {d.batt_percentage:6.1f}% | "
                f"ChargedOnce: {d.has_charged}"
            )
        self.get_logger().info("-" * 40)

    # ===================== INIT =====================
    def assign_initial_roles(self):
        self.fleet[0].role = Role.CENTER
        self.fleet[1].role = Role.FOLLOW
        for i in range(2, NUM_DRONES):
            self.fleet[i].role = Role.SCOUT

# ===================== MAIN =====================
def main():
    rclpy.init()
    node = FleetManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
