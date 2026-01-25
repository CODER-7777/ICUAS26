import numpy as np
import pandas as pd
import cv2
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from shapely.geometry import LineString, Polygon
from cvxopt import matrix, solvers

# --- 1. ENVIRONMENT & TARGET SETUP ---
metadata = {}
try:
    with open("grid_metadata.txt", "r") as f:
        for line in f:
            key, val = line.split()
            metadata[key] = float(val)
except FileNotFoundError:
    print("Error: grid_metadata.txt not found.")

# Constants from Paper Section IV.A [cite: 287, 288]
RESOLUTION = metadata.get('resolution', 0.1)
ORIGIN_X = metadata.get('origin_x', 0.0)
ORIGIN_Y = metadata.get('origin_y', 0.0)
V_MAX = 1.0 # Hardware experiment setting [cite: 343]
A_MAX = 1.0 # Hardware experiment setting [cite: 343]
DC_LIMIT = 3.0 # Strict user-defined constraint

def extract_obstacle_vertices(csv_path):
    grid = pd.read_csv(csv_path, header=None).values.astype(np.uint8)
    contours, _ = cv2.findContours(grid, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    obstacles_vertices = []
    for cnt in contours:
        # Obstacles are modeled as the convex hull of known vertices [cite: 75]
        hull = cv2.convexHull(cnt) 
        if len(hull) < 3: continue
        vertices_m = [[pt[0][0] * RESOLUTION + ORIGIN_X, pt[0][1] * RESOLUTION + ORIGIN_Y] for pt in hull]
        obstacles_vertices.append(np.array(vertices_m))
    return obstacles_vertices

def interpolate_target_path(waypoints, velocity=V_MAX, dt=0.1):
    path = []
    for i in range(len(waypoints) - 1):
        p1, p2 = waypoints[i], waypoints[i+1]
        dist = np.linalg.norm(p2 - p1)
        num_steps = int((dist / velocity) / dt)
        if num_steps > 0:
            for step in range(num_steps):
                path.append((1 - step/num_steps) * p1 + (step/num_steps) * p2)
    return np.array(path)

# --- 2. INITIAL NETWORK DESIGN (Algorithm 1) ---
class MiniEdgeRRTStar:
    def __init__(self, start, goal, obstacles, dc):
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.obstacles = [Polygon(v) for v in obstacles] #cite: 75
        self.dc = dc # Communication range d_c [cite: 75]
        self.kappa = 500.0 # Adjusted penalty for tighter 3m constraints [cite: 103]
        self.tree = {0: {'pos': self.start, 'parent': None, 'cost': 0}}

    def plan(self, max_iter=5000):
        for _ in range(max_iter):
            rand_pt = np.random.uniform(-15, 15, size=2)
            best_parent, min_c = None, float('inf')
            for node_id, node in self.tree.items():
                d = np.linalg.norm(node['pos'] - rand_pt)
                if d <= self.dc: # Range constraint [cite: 106]
                    cost = node['cost'] + d + self.kappa #cite: 103
                    if cost < min_c and self.is_clear(node['pos'], rand_pt):
                        min_c, best_parent = cost, node_id
            
            if best_parent is not None:
                new_id = len(self.tree)
                self.tree[new_id] = {'pos': rand_pt, 'parent': best_parent, 'cost': min_c}
                if np.linalg.norm(rand_pt - self.goal) <= self.dc:
                    if self.is_clear(rand_pt, self.goal):
                        return self.extract_path(new_id)
        return None

    def is_clear(self, p1, p2):
        line = LineString([p1, p2]) #cite: 75
        return not any(line.intersects(obs) and line.intersection(obs).length > 1e-4 for obs in self.obstacles)

    def extract_path(self, end_id):
        path, curr = [self.goal], end_id
        while curr is not None:
            path.append(self.tree[curr]['pos'])
            curr = self.tree[curr]['parent']
        return path[::-1]

# --- 3. DISTRIBUTED MPC (Equation 12) ---
def get_separating_plane(agent_pos, obstacle_vertices):
    # Implements the linear boundary derivation for convex polyhedra [cite: 195, 201]
    dists = [np.linalg.norm(agent_pos - v) for v in obstacle_vertices]
    closest_pt = obstacle_vertices[np.argmin(dists)]
    norm_vec = (agent_pos - closest_pt) / (np.linalg.norm(agent_pos - closest_pt) + 1e-6)
    offset = np.dot(norm_vec, closest_pt + norm_vec * 0.6) # Margin d_m [cite: 199]
    return norm_vec, offset

def solve_mpc_step(current_pos, target_ref, obstacles, params):
    K = params['K'] # Horizon K [cite: 139]
    P = matrix(np.eye(K * 2) * 2.0)
    q = matrix(-2.0 * np.tile(target_ref, K)) #cite: 205, 208
    
    G_list, h_list = [], []
    for obs in obstacles:
        n, b = get_separating_plane(current_pos, obs)
        # Fix 4: Correct (1, 2K) constraint row to match variable dimensions
        G_row = np.zeros((1, K * 2))
        G_row[0, :2] = -n # Simplified: constraint on next position p_1
        G_list.append(G_row)
        h_list.append(-b)
    
    try:
        solvers.options['show_progress'] = False
        # Solve QCQP locally for agent i [cite: 232, 237]
        if not G_list:
            sol = solvers.qp(P, q)
        else:
            sol = solvers.qp(P, q, matrix(np.vstack(G_list)), matrix(h_list))
        return np.array(sol['x'])[:2].flatten()
    except:
        return current_pos # Fallback to predetermined trajectory [cite: 234]

# --- 4. VISUALIZATION & SIMULATION ---
city_obstacles = extract_obstacle_vertices("occupancy_grid_z1.csv")
target_waypoints = np.array([
            [-4.49, -3.44], [-5.49, -7.44], [1.78, -8.31], [4.73, -7.57], 
            [2.50, -1.85], [7.26, 0.92], [5.78, 3.00], [7.92, 7.22], 
            [3.38, 8.08], [1.46, -1.94], [-1.12, -0.48], [-0.10, 8.44], 
            [-7.64, 6.91], [-4.51, 3.11], [-8.22, -1.85], [-2.97, -2.68], 
            [-4.49, -3.44]
        ])
target_traj = interpolate_target_path(target_waypoints)

planner = MiniEdgeRRTStar(start=[0, 0], goal=target_traj[0], obstacles=city_obstacles, dc=DC_LIMIT)
initial_chain = planner.plan()

# Fix 3: Validate initial chain [cite: 88, 255]
if initial_chain is None or len(initial_chain) < 2:
    print("Error: Could not find valid initial 3m chain.")
    exit()

drone_states = np.array(initial_chain[1:])
num_drones = len(drone_states)
mpc_params = {'K': 5, 'h': 0.1, 'dc': DC_LIMIT}
tracking_errors = []

fig, (ax_sim, ax_err) = plt.subplots(1, 2, figsize=(15, 7))
ax_sim.set_xlim(-15, 15); ax_sim.set_ylim(-15, 15)
for obs in city_obstacles: ax_sim.add_patch(plt.Polygon(obs, color='gray', alpha=0.5))
target_plot, = ax_sim.plot([], [], 'r*', markersize=12, label='Target')
drone_plots, = ax_sim.plot([], [], 'bo', markersize=8, label='Drones')
chain_plot, = ax_sim.plot([], [], 'b--', alpha=0.6)
ax_sim.plot(0, 0, 'ks', markersize=10, label='Base Station')
ax_sim.legend()

error_line, = ax_err.plot([], [], 'r-')
ax_err.set_title("Searcher Tracking Error")

def update(frame):
    global drone_states
    if frame >= len(target_traj): return target_plot, drone_plots, chain_plot, error_line
    
    # Fix 1: In-place update for global state mutation
    prev_drone_states = np.copy(drone_states)
    new_drone_states = []
    
    for i in range(num_drones):
        # Neighbor assignment from topology T [cite: 182, 209]
        parent_node = np.array([0.0, 0.0]) if i == 0 else prev_drone_states[i-1]
        child_node = target_traj[frame] if i == num_drones - 1 else prev_drone_states[i+1]
        
        # Reference point balances child/parent neighbors [cite: 209, 224]
        ref = (parent_node + child_node) / 2.0 
        pos = solve_mpc_step(prev_drone_states[i], ref, city_obstacles, mpc_params) 
        new_drone_states.append(pos)
    
    drone_states[:] = np.array(new_drone_states)
    
    # Update Visualization
    target_plot.set_data([target_traj[frame, 0]], [target_traj[frame, 1]])
    drone_plots.set_data(drone_states[:, 0], drone_states[:, 1])
    full_path = np.vstack([[0,0], drone_states, target_traj[frame]])
    chain_plot.set_data(full_path[:, 0], full_path[:, 1])
    
    error = np.linalg.norm(drone_states[-1] - target_traj[frame])
    tracking_errors.append(error)
    error_line.set_data(range(len(tracking_errors)), tracking_errors)
    ax_err.set_xlim(0, max(100, len(tracking_errors)))
    ax_err.set_ylim(0, max(1, max(tracking_errors)))
    
    return target_plot, drone_plots, chain_plot, error_line

ani = FuncAnimation(fig, update, frames=len(target_traj), interval=50, blit=True)
plt.show()