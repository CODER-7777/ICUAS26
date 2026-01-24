import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os

def plot_simulation_path(csv_file="path_log.csv"):
    if not os.path.exists(csv_file):
        print(f"Error: {csv_file} not found.")
        return

    try:
        # Load the CSV
        df = pd.read_csv(csv_file)
        if df.empty:
            print("Error: CSV is empty.")
            return
    except Exception as e:
        print(f"Failed to read CSV: {e}")
        return

    plt.figure(figsize=(10, 8))
    
    # 1. Plot the AGV's full trajectory (Yellow dashed line)
    plt.plot(df['agv_x'], df['agv_y'], 'y--', label='AGV Trace', alpha=0.4, zorder=1)

    # 2. Extract data from the very last row for the communication chain
    last_row = df.iloc[-1]
    agv_final_x = last_row['agv_x']
    agv_final_y = last_row['agv_y']
    
    # 3. Initialize the chain at the Base Station (0,0)
    # The simulation world coordinates are derived from the origin
    chain_x = [0.0]
    chain_y = [0.0]

    # 4. Parse the drone_points string: "x1;y1|x2;y2|..."
    if pd.notna(last_row['drone_points']):
        raw_drones = str(last_row['drone_points']).split('|')
        for drone in raw_drones:
            if ';' in drone:
                coords = drone.split(';')
                chain_x.append(float(coords[0]))
                chain_y.append(float(coords[1]))

    # 5. Connect the last drone to the AGV Position
    chain_x.append(agv_final_x)
    chain_y.append(agv_final_y)

    # 6. Plotting the connections
    # Standard transformation: X_world = X_origin + (i * resolution)
    plt.plot(chain_x, chain_y, color='blue', linestyle='-', linewidth=2, zorder=2, label='Communication Chain')
    
    # Plot Drones (Green circles)
    plt.scatter(chain_x[1:-1], chain_y[1:-1], color='lime', edgecolors='black', s=80, label='Drones', zorder=4)
    
    # Plot Start (Red X) and AGV (Gold circle)
    plt.scatter(0, 0, color='red', marker='X', s=150, label='Base Station', zorder=5)
    plt.scatter(agv_final_x, agv_final_y, color='gold', edgecolors='black', s=150, label='AGV', zorder=5)

    # Label Drones
    for i in range(1, len(chain_x)-1):
        plt.text(chain_x[i]+0.1, chain_y[i]+0.1, f"D{i}", fontsize=9, fontweight='bold')

    # Final plot adjustments
    plt.title(f"Final Drone Waypoints & AGV Connection (T={last_row['timestamp']:.1f}s)")
    plt.xlabel("World X (meters)")
    plt.ylabel("World Y (meters)")
    plt.legend(loc='upper left')
    plt.grid(True, linestyle=':', alpha=0.6)
    plt.axis('equal') 
    
    plt.savefig('simulation_plot.png')
    print("Plot generated successfully as 'simulation_plot.png'.")
    plt.show()

if __name__ == "__main__":
    plot_simulation_path()