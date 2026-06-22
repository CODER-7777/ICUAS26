# Multi-Drone Swarm System — ICUAS'26 Competition

**International Conference on Unmanned Aircraft Systems 2026**
**Team: AerialRobotics-IITK | Jan 2025 – Jun 2025**

---

## Overview

This repository contains the full software stack for a **5-drone swarm system** developed for the ICUAS'26 competition. The system autonomously surveys a 100 m × 100 m × 100 m urban environment, detects ArUco markers affixed to structures, and maintains a dynamic communication relay chain with a moving Autonomous Ground Vehicle (AGV).

**Key Results:**
- Detected **7/8 ArUco markers** in a 100 m³ 3D environment
- Sustained **>70% communication connectivity** with the AGV throughout the mission
- Improved effective flight time by **40%** through intelligent battery scheduling
- Achieved **Global Rank 6** and qualified as the **only Indian team** for the final round in Greece

---

## Table of Contents

1. [Repository Structure](#repository-structure)
2. [System Architecture](#system-architecture)
3. [ROS2 Packages](#ros2-packages)
4. [Core Algorithms](#core-algorithms)
   - [Two-Tier Path Planning](#1-two-tier-path-planning)
   - [Bipartite Drone-Waypoint Matching](#2-bipartite-drone-waypoint-matching)
   - [Pole-Based Search Zone Coverage](#3-pole-based-search-zone-coverage)
   - [Battery Management System (BMS)](#4-battery-management-system-bms)
   - [Collision Avoidance via Repulsion Fields](#5-collision-avoidance-via-repulsion-fields)
   - [Multi-Drone ArUco Detection & Fusion](#6-multi-drone-aruco-detection--fusion)
5. [Data Structures & Types](#data-structures--types)
6. [ROS2 Communication Interface](#ros2-communication-interface)
7. [Mission State Machine](#mission-state-machine)
8. [Configuration & Parameters](#configuration--parameters)
9. [Build & Launch](#build--launch)
10. [Visualization](#visualization)
11. [Performance Characteristics](#performance-characteristics)
12. [Architectural Decisions](#architectural-decisions)

---

## Repository Structure

```text
icuas26_competition/
├── workspace/
│   └── src/
│       ├── aruco_mission_cpp/          # Multi-drone ArUco detection & pose fusion
│       │   ├── src/
│       │   │   ├── aruco_mission_node.cpp
│       │   │   └── utils.cpp
│       │   ├── CMakeLists.txt
│       │   └── package.xml
│       ├── graph_traversal/            # Core swarm planning (BFS, A*, BMS, matching)
│       │   ├── src/
│       │   │   ├── types.hpp           # Shared types: idx, DroneBatteryState, AStarNode
│       │   │   ├── utils.hpp / utils.cpp
│       │   │   ├── bfs.cpp             # Relay-chain BFS path planner
│       │   │   ├── occupancy_grid.cpp  # OctoMap -> 2D occupancy slice
│       │   │   ├── prediction.cpp      # OctoMap service->topic bridge
│       │   │   ├── search_manager.hpp / search_manager.cpp  # Pole detection & view zones
│       │   │   ├── swarm_planner.hpp   # SwarmPlanner class declaration
│       │   │   ├── swarm_planner_init.cpp    # Constructor, parameter loading
│       │   │   ├── swarm_planner_grid.cpp    # A*, LOS, occupancy helpers
│       │   │   ├── swarm_planner_mission.cpp # State dispatch, matching, job assignment
│       │   │   ├── swarm_planner_bms.cpp     # Battery FSM
│       │   │   ├── swarm_planner_rth.cpp     # Sequential return-to-home logic
│       │   │   └── swarm_planner_control.cpp # Repulsion, speed limiting, 20 Hz publish
│       │   ├── CMakeLists.txt
│       │   └── package.xml
│       ├── debug_tools/                # Mission event logging
│       │   └── src/
│       │       └── debug_logger_node.cpp
│       └── swarm_viz/                  # Real-time Foxglove/RViz2 visualization
├── config/
│   ├── charging.yaml                   # Charging area coordinates
│   └── aruco_parameters.yaml          # Per-drone camera calibration
├── launch/                            # ROS2 launch files
├── worlds/                            # Gazebo simulation environments
└── resume_points.tex                  # Competition project summary (LaTeX)
```

---

## System Architecture

The system is composed of four cooperating ROS2 nodes running concurrently:

```text
+------------------------------------------------------------------+
|                          ENVIRONMENT                             |
|   /AGV/pose ------------------------------------------+          |
|   OctoMap service ----------------------+             |          |
+----------------------------------------|--------------+----------+
                                         |             |
                              +----------v----------+  |
                              |   occupancy_grid    |  |
                              |   (2D slice)        |  |
                              +----------+----------+  |
                                         |             |
                              +----------v----------+  |
                              |   bfs (relay)       |<-+
                              |   /waypoint_array   |
                              +----------+----------+
                                         |
          +------------------------------v------------------------------+
          |                  SwarmPlanner (move)                        |
          |  +------------------+   +-------------------------------+   |
          |  | Bipartite        |   |  SearchManager                |   |
          |  | Matching         |   |  (pole zones)                 |   |
          |  +------------------+   +-------------------------------+   |
          |  +------------------+   +-------------------------------+   |
          |  | A* per drone     |   |  Battery FSM                  |   |
          |  +------------------+   +-------------------------------+   |
          |  +----------------------------------------------------------+|
          |  |  Repulsion + Speed Limiter (20 Hz)                      ||
          |  +----------------------------------------------------------+|
          +------------------------------+------------------------------+
                                         | /cf_X/cmd_position
          +------------------------------v------------------------------+
          |               aruco_mission_node                            |
          |         5 parallel camera streams                           |
          |         OpenCV ArUco + pose fusion                          |
          +-------------------------------------------------------------+
```

---

## ROS2 Packages

### `aruco_mission_cpp`

**Node:** `aruco_mission_node`

Processes five simultaneous camera streams to detect ArUco markers, fuses multi-drone observations, and publishes verified marker locations to the competition scoring interface.

**Key design choices:**
- `MultiThreadedExecutor` with 6 threads and per-drone `Reentrant` callback groups allow all five image pipelines to run in parallel
- Frame throttling at 10 Hz per drone prevents CPU saturation without degrading detection latency
- Uses `DICT_5X5_250` dictionary with 0.25 m physical marker size

---

### `graph_traversal`

Contains four executables compiled from a shared source tree:

| Executable | Entry Point | Role |
|---|---|---|
| `prediction` | `prediction.cpp` | Polls OctoMap service; republishes as topic |
| `occupancy` | `occupancy_grid.cpp` | Extracts 2D horizontal slice from OctoMap |
| `bfs` | `bfs.cpp` | Relay-chain path planner (BFS + LOS reduction) |
| `move` | `swarm_planner_*.cpp` | Main swarm coordinator (A*, BMS, matching, control) |

---

### `debug_tools`

**Node:** `debug_logger_node`

Subscribes to all mission-critical topics and writes time-stamped events (state transitions, detections, battery events) to `/root/workspace/log/mission_log.txt`.

---

### `swarm_viz`

**Node:** `viz_node`

Publishes `visualization_msgs/MarkerArray` messages for real-time monitoring in Foxglove Studio or RViz2:
- Drone positions as labeled spheres
- Per-drone trajectory trails (`nav_msgs/Path`)
- Communication graph edges color-coded by LOS status (green = clear, red = blocked, yellow = out of range)

---

## Core Algorithms

### 1. Two-Tier Path Planning

The system uses a hierarchical planning architecture that separates global relay routing from per-drone local navigation.

#### Tier 1 — BFS Relay-Chain Planner (`bfs.cpp`, ~3 Hz)

The relay-chain planner runs as an independent node and computes a sparse chain of waypoints connecting the fixed base station at (0, 0) to the moving AGV, such that consecutive waypoints are within communication range.

**Algorithm:**

1. **8-connected BFS** on the inflated 2D occupancy grid from base to AGV position
2. **Waypoint reduction via greedy line-of-sight (LOS) jumps:**
   - Starting from the base, scan forward along the BFS path
   - Jump to the farthest waypoint reachable via Bresenham rasterization with a safety buffer (cell occupancy < 50 and all 4-neighbors < 50)
   - Accept the jump only if the 3D distance to the AGV segment uses the full `max_dist` communication range
   - First hop (base to relay) and last hop (relay to AGV) use 3D distance; intermediate hops use 2D projected distance
3. **AGV Loop Detection:**
   - Tracks whether the AGV has moved >3 m from its starting position
   - When the AGV returns within `LOOP_DETECTION_RADIUS = 1 m` of start after having left, the loop counter increments
   - `z_target` is raised by one altitude step per loop, causing the relay chain to climb progressively (enabling 3D coverage of the environment over multiple passes)

**Output:** `/drone/waypoint_array` (`geometry_msgs/PoseArray`) — the computed relay chain waypoints.

#### Tier 2 — A\* Per-Drone Planner (`swarm_planner_grid.cpp`, 4 Hz)

Each drone independently plans a collision-free path from its current position to its assigned waypoint using A\*.

**Implementation details:**
- **Heuristic:** Chebyshev distance — `h(n) = max(|di|, |dj|)` — admissible for 8-connected grids
- **Expansion:** 8-connected neighbors, skipping inflated obstacle cells
- **Lazy deduplication:** If a node is popped from the min-heap with a g-score higher than the recorded minimum, it is discarded without expansion (avoids explicit closed-set with a hash map)
- **Thread-local buffers with generation tokens:** Each planning thread reuses a pre-allocated visited/cost array; a monotonically increasing generation counter marks cells as "belonging to the current search" without zeroing the buffer between calls — eliminates per-call memory allocation overhead
- **Path smoothing:** After A\* completes, a greedy LOS pass removes intermediate waypoints that are directly visible from a predecessor

---

### 2. Bipartite Drone-Waypoint Matching

At each planning cycle, available drones must be optimally assigned to relay-chain waypoints. The problem is formulated as a minimum-bottleneck bipartite matching.

**Algorithm:** Binary search over cost threshold + DFS augmenting paths

```text
1. Build cost matrix C[d][w] = 3D Euclidean distance from drone d to waypoint w
2. Collect all unique distances into sorted array D
3. Binary search on D to find minimum threshold T such that a perfect matching exists
4. For each candidate T:
     - Build adjacency: drone d can reach waypoint w iff C[d][w] <= T
     - Run DFS augmenting-path matching (Hungarian-style)
     - Accept T if |matching| = min(|drones|, |waypoints|)
5. Return final assignment from the accepted matching
```

**Complexity:** O(log |D| * V * E) where V = number of drones, E = number of (drone, waypoint) pairs within threshold, |D| = number of unique distances.

**Effect:** Minimizes the maximum travel distance any single drone must cover to reach its relay position, which reduces gap-formation in the chain under AGV movement.

Unmatched drones (when |drones| > |waypoints|) are assigned to search roles via the SearchManager.

---

### 3. Pole-Based Search Zone Coverage (`search_manager.cpp`)

The `SearchManager` class drives unassigned drones to systematically inspect vertical structures (poles, columns) where ArUco markers are placed.

#### Pole Detection from OctoMap

1. Extract a horizontal slab at 0.2–0.5 m above ground, 0.3 m thick, from the OctoMap
2. Project occupied voxels onto the 2D grid to form a binary footprint image
3. Run a **4-connected flood-fill** (BFS) to identify connected components
4. Filter components:
   - Reject components with fewer than `min_pole_cells_ = 4` cells (noise)
   - Reject components whose bounding box extent exceeds `max_pole_extent_ = 1.5 m` in either axis (walls, fences)
5. Compute centroid `(cx, cy)` and approximate radius from surviving components — `PoleInfo {cx, cy, radius, z_bottom, z_top}`

#### View Zone Generation

For each detected pole, 8 view zones are generated:

- **4 cardinal directions:** East, North, West, South
- **2 heights:** 25% and 75% of pole height
- **Zone position:** `(cx + standoff * direction_x, cy + standoff * direction_y, z_height)`
- **Zone yaw:** facing pole center — `atan2(cy - zone_y, cx - zone_x)`

where `standoff_ = 1.5 m` is the camera standoff distance.

#### Zone Selection for Idle Drones

Each idle drone is assigned the nearest unvisited zone that satisfies:
- Line-of-sight from the zone to any relay anchor (base station or an active chain drone)
- Falls back to a "shadow position" (interpolated between base and AGV) if no reachable zone exists

#### Visit Tracking

A zone is marked visited when a drone satisfies both:
1. 3D distance to zone center < `visit_radius_ = 1.5 m`
2. Dot product of (drone to zone_center) with the expected approach direction > 0 (correct side of pole)

This prevents marking a zone visited when flying past it on the wrong side.

---

### 4. Battery Management System (BMS)

The BMS manages charging logistics for all drones to maximize fleet uptime without endangering mission continuity.

#### Per-Drone State Machine

```text
              +------------------------------+
              |           MISSION            |
              |  (normal flight & coverage)  |
              +-------------+----------------+
                            |
                            | battery <= 70%
                            | AND slot available
                            | AND concurrency < MAX_CHARGING_DRONES
                            v
              +------------------------------+
              |      is_going_to_charge      |
              |  (flying to charging slot)   |
              +-------------+----------------+
                            |
                            | distance to slot <= 0.1 m
                            v
              +------------------------------+
              |        is_charging           |
              |  (landed on charging pad)    |
              +-------------+----------------+
                            |
                            | battery >= 88%
                            v
              +------------------------------+
              |     is_leaving_charger       |
              |  (clearance maneuver)        |
              +-------------+----------------+
                            |
                            | moved > 1.5 m from slot
                            | OR 5 s timeout
                            v
              +------------------------------+
              |           MISSION            |
              +------------------------------+
```

**Key parameters:**
- `MAX_CHARGING_DRONES = 2` — at most 2 drones charging simultaneously
- Charging slots are placed diagonally at ±0.8 m from charging area center (2.2 m separation, exceeding the 1.4 m physical clearance requirement)
- Threshold to trigger charging: battery <= 70%
- Threshold to resume mission: battery >= 88%
- Emergency RTH: any drone battery < 25% triggers entire swarm transition to `RETURN_TO_HOME`

#### Smart Recall Logic

If the number of available (non-charging) drones falls below the number of relay waypoints, the BMS considers recalling the highest-battery charging drone IF:
- It has gained >= 10% charge since starting to charge, OR
- Zero drones are available for relay coverage

This prevents mission collapse during simultaneous high-demand periods without forcing premature charging termination.

---

### 5. Collision Avoidance via Repulsion Fields

The control loop (20 Hz) applies a potential-field repulsion step to each drone's next waypoint before publishing the position command.

**Algorithm:**

```text
for each other drone j:
    d = distance(drone_i, drone_j)
    if d < safe_radius:
        strength = gain * (safe_radius - d) / d
        push += (unit_vector from j to i) * strength

if |push| > MAX_REPULSION_STEP:
    push = normalize(push) * MAX_REPULSION_STEP

adjusted_target = next_waypoint + push
```

**Parameters:**
- `safe_radius = 0.7 m` during mission; `0.0 m` during RTH
- `MAX_REPULSION_STEP = 1.0 m` — clamps repulsion displacement to prevent overcorrection

#### Speed Limiting

After applying repulsion, the adjusted target is checked against a maximum speed constraint:

```text
step_vector = adjusted_target - current_pose
max_step = max_speed * control_dt   # e.g., 12.0 m/s * 0.05 s = 0.6 m
if |step_vector| > max_step:
    adjusted_target = current_pose + normalize(step_vector) * max_step
```

This ensures position commands never demand velocity above `max_speed`, preventing hardware over-speed faults.

---

### 6. Multi-Drone ArUco Detection & Fusion

#### Detection Pipeline (per drone, 10 Hz)

1. Receive `sensor_msgs/Image` from `/cf_X/image`
2. Convert to OpenCV `Mat` (BGR)
3. Run `cv::aruco::detectMarkers()` with `DICT_5X5_250` dictionary
4. For each detected corner set, call `cv::aruco::estimatePoseSingleMarkers()` using per-drone calibration from `sensor_msgs/CameraInfo`
5. Transform the marker pose from camera frame to world frame using the drone's current pose from `/cf_X/pose`

#### Multi-Drone Fusion

Observations from different drones for the same marker ID are aggregated into a weighted average:

```text
weight_i = 1.0 / (distance_i + 0.1)

fused_x = sum(weight_i * obs_i.x) / sum(weight_i)
fused_y = sum(weight_i * obs_i.y) / sum(weight_i)
fused_z = sum(weight_i * obs_i.z) / sum(weight_i)
```

**Outlier rejection:** observations where the distance from the weighted mean exceeds 2.5 standard deviations are discarded before computing the final estimate.

#### Publish Strategy

- **Landing markers (IDs 11–20):** published immediately on detection (needed for in-mission landing decisions)
- **Scoring markers (all other IDs):** batched and published once when `RTH_STATE = true` (avoids duplicate submissions during flight)

---

## Data Structures & Types

Defined in `workspace/src/graph_traversal/src/types.hpp`:

```cpp
// 2D grid coordinate
struct idx {
    int i, j;
};

// Per-drone battery and charging state
struct DroneBatteryState {
    float percentage;
    bool has_charged;               // one-shot: prevents re-triggering after RTH
    bool is_charging;
    bool is_going_to_charge;
    bool is_leaving_charger;
    rclcpp::Time start_leaving_time;
    int assigned_slot;              // index into charging_slots_ array
    float start_charge_percentage;  // to compute gained charge for recall logic
};

// A* open-set entry
struct AStarNode {
    int idx_flat;                   // linearized grid index: i * width + j
    int f_score;
    bool operator>(const AStarNode& other) const;  // for min-heap
};

// Top-level mission state
enum class SwarmState {
    TAKEOFF,
    MISSION,
    RETURN_TO_HOME
};

// Detected pole from OctoMap flood-fill
struct PoleInfo {
    double cx, cy;       // centroid in world coordinates
    double radius;       // approximate footprint radius
    double z_bottom;     // lowest occupied voxel
    double z_top;        // highest occupied voxel
};

// Generated view zone for ArUco coverage
struct ViewZone {
    double x, y, z;      // world position
    double yaw;          // heading to face pole center
    int pole_idx;        // parent pole index
    int dir_idx;         // 0=E, 1=N, 2=W, 3=S
    int height_idx;      // 0=25%, 1=75%
    bool visited;        // marked true when drone reaches within visit_radius_
};
```

---

## ROS2 Communication Interface

### Subscriptions (SwarmPlanner / bfs)

| Topic | Type | Publisher | Purpose |
|---|---|---|---|
| `/AGV/pose` | `geometry_msgs/Point` | Environment | AGV position for relay planning |
| `/cf_X/pose` | `geometry_msgs/PoseStamped` | Crazyflie driver | Drone position (100 Hz) |
| `/cf_X/battery_status` | `sensor_msgs/BatteryState` | Crazyflie driver | Battery percentage |
| `/drone/waypoint_array` | `geometry_msgs/PoseArray` | bfs node | Relay chain waypoints |
| `/mission/z_target` | `std_msgs/Float64` | bfs node | Current altitude target |

### Publications (SwarmPlanner)

| Topic | Type | QoS | Purpose |
|---|---|---|---|
| `/cf_X/cmd_position` | `crazyflie_interfaces/Position` | Reliable, 20 Hz | Per-drone position commands |
| `RTH_STATE` | `std_msgs/Bool` | Transient-local | Signals RTH transition to ArUco node |
| `/mission_done` | `std_msgs/Bool` | Transient-local | Signals mission completion |

### Publications (bfs)

| Topic | Type | Purpose |
|---|---|---|
| `/drone/waypoint_array` | `geometry_msgs/PoseArray` | Relay chain waypoints (~3 Hz) |
| `/mission/z_target` | `std_msgs/Float64` | Altitude (updated on AGV loop) |

### Publications (ArUco node)

| Topic | Type | Purpose |
|---|---|---|
| `/target_found` | `icuas25_msgs/TargetInfo` | Detected marker `{id, location.x/y/z}` |

### Services Called

| Service | Type | Called By |
|---|---|---|
| `/cf_X/takeoff` | `crazyflie_interfaces/Takeoff` | SwarmPlanner (TAKEOFF state) |
| `/cf_X/land` | `crazyflie_interfaces/Land` | SwarmPlanner (RTH landing) |
| `octomap_binary` | `octomap_msgs/GetOctomap` | prediction node |

---

## Mission State Machine

The top-level `SwarmState` FSM governs all drone behaviors:

### TAKEOFF

```text
1. Call /cf_X/takeoff service for all N drones (target height: 1.0 m)
2. Poll /cf_X/pose at 4 Hz
3. Transition to MISSION when all drones reach z_target +/- 0.15 m
```

### MISSION

Executed at 4 Hz (planning) and 20 Hz (control):

```text
Planning cycle (4 Hz):
  1. Snapshot occupancy grid, waypoints, drone poses (under mutex)
  2. AGV loop detection -> update z_target if new loop
  3. SearchManager.markVisited() for all drone positions
  4. BMS FSM transitions for all drones (handleBatteryLogic)
  5. Partition drones: available vs. charging
  6. Emergency check: any battery < 25% -> RETURN_TO_HOME
  7. Bipartite matching: assign available drones to relay waypoints
  8. Job assignment:
       - Matched drone   -> role=CHAIN_COMPONENT, target=assigned waypoint
       - Unmatched drone -> role=SEARCH, target=nearest unvisited view zone
       - Charging drone  -> role=RTH/LANDING, target=charging slot or clearance point
  9. A* planning: per-drone path from current pose to target
 10. Yaw override: search zones face pole; charging zones face 0 degrees
 11. Atomic swap of active_commands_ (mutex protected)

Control cycle (20 Hz):
  1. Snapshot active_commands_ and drone poses
  2. For each drone:
       a. Select next waypoint (index 1 if path length > 1, else 0)
       b. Apply repulsion from neighbors within safe_radius
       c. Apply speed limiter
       d. Publish to /cf_X/cmd_position
```

### RETURN_TO_HOME

```text
1. Sort drones by battery percentage (ascending -- lowest first)
2. For drone i (from 0 to N-1):
     a. Drones 0..i-1: already landed (idle)
     b. Drone i: A* plan to initial home position, publish commands
     c. Drones i+1..N-1: hover in place
     d. Transition to next drone when drone i lands (altitude < threshold)
3. Call /cf_X/land service for each drone in sequence
4. Publish /mission_done = true (transient-local)
```

RTH is triggered by:
- Any drone battery < 25% (emergency)
- Manual trigger (configurable)

---

## Configuration & Parameters

### Environment Variables

```bash
export NUM_ROBOTS=5         # number of drones (default: 5)
export COMM_RANGE=70        # communication range in meters (default: 70)
export CHARGING_FILE=5      # charging config filename suffix (default: "5")
```

### `config/charging.yaml`

```yaml
charging_area:
  upper_left: [-1.0, 1.0]   # [x, y] in meters
  down_right:  [ 1.0, -1.0]
```

### `config/aruco_parameters.yaml`

Per-drone camera calibration (intrinsics, distortion coefficients), marker dictionary (`DICT_5X5_250`), and physical marker size (0.25 m).

### Runtime Parameters (SwarmPlanner)

| Parameter | Default | Description |
|---|---|---|
| `inflation_radius` | 0.3 m | Obstacle clearance buffer for grid inflation |
| `z_target` | 1.0 m | Initial mission altitude |
| `max_speed` | 12.0 m/s | Maximum drone velocity |

### Runtime Parameters (SearchManager)

| Parameter | Default | Description |
|---|---|---|
| `standoff_` | 1.5 m | Camera standoff distance from pole center |
| `visit_radius_` | 1.5 m | Proximity threshold to mark zone visited |
| `min_pole_cells_` | 4 cells | Minimum flood-fill component size (noise rejection) |
| `max_pole_extent_` | 1.5 m | Maximum component extent (wall/fence rejection) |

---

## Build & Launch

### Prerequisites

- ROS2 Humble or later
- OpenCV 4.x (with ArUco contrib module)
- `octomap_msgs`, `crazyflie_interfaces`, `icuas25_msgs` packages
- Gazebo (for simulation)

### Build

```bash
cd workspace
colcon build --symlink-install
source install/setup.bash
```

### Launch Individual Nodes

```bash
# OctoMap bridge (run first)
ros2 run graph_traversal prediction

# 2D occupancy grid extraction
ros2 run graph_traversal occupancy

# BFS relay-chain planner
ros2 run graph_traversal bfs

# Main swarm coordinator
ros2 run graph_traversal move

# ArUco detection (all drones)
ros2 run aruco_mission_cpp aruco_mission_node

# Visualization
ros2 run swarm_viz viz_node

# Debug logging
ros2 run debug_tools debug_logger_node
```

### Full Mission Launch

```bash
ros2 launch icuas26_competition mission.launch.py num_robots:=5 comm_range:=70
```

---

## Visualization

The `swarm_viz` node publishes the following topics for Foxglove Studio or RViz2:

| Topic | Type | Content |
|---|---|---|
| `/swarm_viz/drones` | `visualization_msgs/MarkerArray` | Drone positions as labeled blue spheres |
| `/swarm_viz/comm_graph` | `visualization_msgs/MarkerArray` | Relay edges (green=LOS clear, red=LOS blocked, yellow=out of range) |
| `/swarm_viz/trail/cf_X` | `nav_msgs/Path` | Per-drone trajectory trail (last 50 poses) |
| `/swarm_viz/occupancy_slice` | `nav_msgs/OccupancyGrid` | 2D planning grid (transient-local) |

---

## Performance Characteristics

### Timing Budget

| Component | Frequency | Notes |
|---|---|---|
| Control loop | 20 Hz (50 ms) | Repulsion + speed limit + publish |
| Planning loop | 4 Hz (250 ms) | BMS + matching + A* x 5 drones |
| BFS relay | ~3 Hz | AGV tracking + LOS reduction |
| ArUco detection | 10 Hz per drone | Frame-throttled to prevent CPU saturation |
| OctoMap poll | 2 s interval | One-shot on first success |

### Computational Complexity

| Algorithm | Complexity | Typical Size |
|---|---|---|
| A* (per drone) | O(W*H*log(W*H)) | 100x100 grid |
| Bipartite matching | O(log\|D\| * V * E) | V=5 drones, E<=8 waypoints |
| Pole flood-fill | O(W*H) | 100x100 grid |
| Bresenham LOS | O(max(W,H)) | per check |

### Memory Footprint

| Structure | Size |
|---|---|
| Thread-local A* buffer | ~40 KB per thread (100x100 x int) |
| Drone trail (viz) | ~600 B per drone (50 x PoseStamped) |
| OctoMap (in-memory) | Variable; shared_ptr to prevent copies |

---

## Architectural Decisions

**Reentrant callback groups:** Five image subscriber callbacks run truly in parallel within `MultiThreadedExecutor`, enabling real-time ArUco detection across all drones without inter-drone blocking.

**Shared pointer for occupancy grid:** Planning and control threads snapshot the grid via `shared_ptr` reference count bump rather than copying the 100x100 array, eliminating copy overhead at 4 Hz replanning.

**Generation-token trick for A\*:** Thread-local visited/cost buffers are reused across replanning calls. A monotonically increasing generation counter tags which cells belong to the current search, avoiding `memset` or `fill` between calls.

**Transient-local QoS for RTH_STATE:** The ArUco node may start after the swarm planner. Transient-local durability ensures the ArUco node receives the current RTH state upon connection without the planner needing to re-publish.

**Binary-search bipartite matching over Hungarian algorithm:** For 5 drones and <=8 waypoints, the O(log D * VE) binary search approach has lower constant factors than the O(V^3) Hungarian algorithm and produces the minimum bottleneck assignment, which is the correct objective for relay-chain coverage.

**Sequential RTH:** Drones land one at a time rather than all simultaneously. This prevents collision on the landing pad and avoids a thundering-herd scenario where all drones attempt to reach the same ground region at once.

**Smart recall over fixed time-slicing:** Battery-aware recall (>=10% gained) allows the BMS to make runtime decisions based on actual mission state rather than following a predetermined schedule, which is brittle to unexpected AGV behavior or obstacle avoidance detours.

**Two-tier planning separation:** The BFS relay planner and the A* per-drone planner run as separate nodes communicating over topics. This decouples relay topology updates (driven by AGV movement) from drone-level replanning (driven by local obstacles and BMS transitions), allowing each to run at its natural frequency without coupling.

---

*Developed by AerialRobotics-IITK for the International Conference on Unmanned Aircraft Systems 2026 (ICUAS'26).*
