import matplotlib.pyplot as plt
import csv

# Set matplotlib to open in an interactive window
# %matplotlib qt 
# (Above line is for notebooks, not needed for script)

print("Plotter: Reading vertex data...")
# 1. Read all vertex coordinates
vertices = {}
with open('vertices.csv', 'r') as f:
    reader = csv.reader(f)
    next(reader) # Skip header
    for row in reader:
        # id, x, y, z
        vertices[int(row[0])] = (float(row[1]), float(row[2]), float(row[3]))

print("Plotter: Reading final path data...")
# 2. Read the final path
path = []
with open('final_path.txt', 'r') as f:
    for line in f:
        path.append(int(line.strip()))

if not path:
    print("Plotter: Error, final_path.txt is empty.")
    exit()

print(f"Plotter: Plotting path with {len(path)} steps...")

# 3. Unzip the path coordinates for plotting
x_coords = []
y_coords = []
z_coords = []
for v_id in path:
    coord = vertices[v_id]
    x_coords.append(coord[0])
    y_coords.append(coord[1])
    z_coords.append(coord[2])

# 4. Plot the 3D path
fig = plt.figure(figsize=(12, 12))
ax = fig.add_subplot(111, projection='3d')

# Plot the path as a line
ax.plot(x_coords, y_coords, z_coords, marker='o', markersize=1, linestyle='-', label='Drone Path')

# Plot start and end points
ax.plot([x_coords[0]], [y_coords[0]], [z_coords[0]], 'go', markersize=10, label='Start (Origin)') # Start
ax.plot([x_coords[-1]], [y_coords[-1]], [z_coords[-1]], 'rs', markersize=10, label='End') # End

ax.set_xlabel('X Coordinate')
ax.set_ylabel('Y Coordinate')
ax.set_zlabel('Z Coordinate')
ax.set_title('3D Chinese Postman Tour')
ax.legend()

print("Plotter: Showing plot...")
plt.show()