import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from crazyflie_py import Crazyswarm
import time

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor_node')
        self.current_voltage = 0.0
        # Hardcoding cf_1 as confirmed by your server handshake logs
        self.subscription = self.create_subscription(
            BatteryState,
            '/cf_1/battery_status',
            self.battery_callback,
            10)

    def battery_callback(self, msg):
        self.current_voltage = msg.voltage

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    
    if len(swarm.allcfs.crazyflies) == 0:
        print("Error: No Crazyflies detected!")
        return

    # Use the first available drone
    
    cf = swarm.allcfs.crazyflies[2]
    print("--- Starting Mission: cf_1 ---")

    # Initialize battery monitoring
    monitor = BatteryMonitor()

    # 1. Takeoff to 1.0m
    print("Action: Taking off...")
    cf.takeoff(targetHeight=1.0, duration=2.0)
    timeHelper.sleep(4.0)

    # 2. Hover for 200  seconds
    print("Action: Hovering...")
    timeHelper.sleep(200.0)

    # 3. Go to Charging Area Center
    # Center of [-1.5, 1.5] and [1.5, -1.5] is [0, 0]
    print("Action: Moving to Charging Area [0, 0]...")
    cf.goTo(goal=[0.0, 0.0, 1.0], yaw=0.0, duration=4.0)
    timeHelper.sleep(5.0)

    # 4. Land for Charging
    print("Action: Landing to charge...")
    cf.land(targetHeight=0.0, duration=3.0)
    timeHelper.sleep(5.0)

    print("Mission Complete. Monitoring Battery (Voltage should rise toward 4.2V)...")
    try:
        while rclpy.ok():
            rclpy.spin_once(monitor, timeout_sec=0.1)
            # Use \r to update the same line in terminal
            print(f"Current Voltage: {monitor.current_voltage:.4f}V", end='\r')
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()