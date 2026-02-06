#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import numpy as np

class GridPlotter(Node):
    def __init__(self):
        super().__init__('grid_plotter')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/reward_grid',
            self.grid_callback,
            10)
        
        self.drone_poses = {}
        for i in range(1, 6):
            topic = f'/cf_{i}/pose'
            self.create_subscription(
                PoseStamped,
                topic,
                lambda msg, drone_id=i: self.pose_callback(msg, drone_id),
                10
            )

        self.grid_data = None
        self.grid_info = None
        self.new_grid = False
        self.get_logger().info('GridPlotter started. Listening for grid and poses...')

    def grid_callback(self, msg):
        self.grid_data = np.array(msg.data, dtype=np.int8)
        self.grid_info = msg.info
        self.new_grid = True

    def pose_callback(self, msg, drone_id):
        self.drone_poses[drone_id] = (msg.pose.position.x, msg.pose.position.y)

def main(args=None):
    rclpy.init(args=args)
    node = GridPlotter()
    
    try:
        plt.ion()
        fig, ax = plt.subplots(figsize=(8, 8))
        img = None
        drone_markers = None
        
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            
            if node.new_grid and node.grid_data is not None:
                node.new_grid = False
                data = node.grid_data
                info = node.grid_info
                
                width = info.width
                height = info.height
                res = info.resolution
                origin_x = info.origin.position.x
                origin_y = info.origin.position.y
                
                grid_2d = data.reshape((height, width))
                
                # Calculate dynamic range for visualization
                min_val = np.min(grid_2d)
                max_val = np.max(grid_2d)
                
                # If all values are the same, use a default range to avoid errors
                if min_val == max_val:
                    min_val -= 1
                    max_val += 1

                extent = [origin_x, origin_x + width * res, 
                          origin_y, origin_y + height * res]
                
                if img is None:
                    img = ax.imshow(grid_2d, cmap='RdYlGn', origin='lower', 
                                    extent=extent, vmin=min_val, vmax=max_val)
                    colorbar = plt.colorbar(img, ax=ax, label='Reward')
                    ax.set_title("Reward Grid with Drones")
                    ax.set_xlabel("X [m]")
                    ax.set_ylabel("Y [m]")
                else:
                    img.set_data(grid_2d)
                    img.set_extent(extent)
                    img.set_clim(vmin=min_val, vmax=max_val)
            
            # Update drone markers (always update if they exist)
            if node.drone_poses:
                xs = [pos[0] for pos in node.drone_poses.values()]
                ys = [pos[1] for pos in node.drone_poses.values()]
                
                if drone_markers:
                    drone_markers.remove()
                
                drone_markers = ax.scatter(xs, ys, c='blue', s=100, marker='X', label='Drones', edgecolors='white')
                # Optional: Add annotations? Too slow for loop maybe.

            plt.draw()
            plt.pause(0.05)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.close('all')

if __name__ == '__main__':
    main()
