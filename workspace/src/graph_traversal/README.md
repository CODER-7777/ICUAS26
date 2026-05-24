# graph_traversal Package

ROS 2 C++ package for drone relay-chain planning using OctoMap, BFS, A*, and swarm management. Designed for the ICUAS'26 competition.

## System Architecture

4 executables (nodes) working together:

```
prediction  ──► occupancy_grid  ──► bfs  ──► move
(OctoMap        (2D slice           (BFS chain    (Swarm mission
 service        at Z height)         planner)      manager)
 → topic)
```

Plus `utils` (shared helpers) and `plot.py` (offline visualization).

---

## 1. `prediction.cpp` — `OctomapServiceToTopic` (59 lines)

**Purpose:** Bridges the OctoMap *service* (`octomap_binary`) to a *topic* (`octomap_binary_topic`). Retries every 2s until the first successful call, then cancels the timer (one-shot).

| Signature | Line | What it does |
|---|---|---|
| `void timerCallback()` | 26 | Every 2s, checks service readiness, calls `async_send_request`; on success, publishes the binary map and **cancels the timer** |
| `int main(int argc, char** argv)` | 53 | Spins the node |

---

## 2. `occupancy_grid.cpp` — `OctomapToPlane` (131 lines)

**Purpose:** Extracts a **2D horizontal slice** from the OctoMap at a given Z height and publishes as `nav_msgs::msg::OccupancyGrid` on `/map_slice`. Saves `latest_plane_slice.png`. Retries every `retry_period` until the first successful call, then cancels the timer (one-shot).

| Signature | Line | What it does |
|---|---|---|
| `void timerCallback()` | 32 | Every `retry_period` seconds (default 2.0), calls OctoMap service |
| `void handleResponse(rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedFuture future)` | 44 | Deserializes OctoMap → iterates leaf nodes in a Z-thick bounding box → marks occupied cells in a grid → publishes + saves PNG → **cancels the timer** |
| `int main(int argc, char** argv)` | 126 | Spins the node |

---

## 3. `bfs.cpp` — `OctomapBFSPlanner` (440 lines)

**Purpose:** The **relay-chain path planner**. Plans a chain of waypoints from base station (0,0) to the AGV using BFS (8-connected), then applies line-of-sight waypoint reduction. Publishes to `/drone/waypoint_array`.

| Signature | Line | What it does |
|---|---|---|
| `OctomapBFSPlanner()` | 27 | Declares params (`z_target`, `inflation_radius`, `max_dist`, `frame_id`); creates AGV pose subscriber, waypoint publisher, z_target publisher, OctoMap service client, init timer (cancels itself after first map), main plan loop timer at ~3 Hz |
| `void timerLoop()` | 105 | Main loop: captures AGV pose, detects AGV loops (return to start → raises altitude), calls `runPlanningPipeline()`, publishes waypoints and z_target |
| `void fetchOctomapOnce()` | 158 | Calls OctoMap service, deserializes tree, computes `z_step_` from map `z_max` (divides into ~10 steps), generates initial occupancy grid, sets `map_ready_` |
| `void publishPoseArray(const std::vector<geometry_msgs::msg::Point>& path)` | 205 | Converts path points → `PoseArray` and publishes |
| `std::vector<idx> bfs(idx start, idx end, const nav_msgs::msg::OccupancyGrid& map)` | 219 | **BFS** (8-connected) on occupancy grid; returns vector of `idx` from start to end; early exit if start/end out of bounds |
| `static double dist3D(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b)` | 260 | 3D Euclidean distance |
| `std::vector<geometry_msgs::msg::Point> runPlanningPipeline(const nav_msgs::msg::OccupancyGrid& map, geometry_msgs::msg::Point target)` | 265 | Converts world → grid coords, calls `bfs()`, greedily reduces waypoints using LOS check (Bresenham) within `max_dist`; first/last segments use 3D distance (base at z=0, AGV on ground), intermediate use 2D; enforces `MIN_WAYPOINT_SEP = 2.0m` |
| `bool hasLineOfSight(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b, const nav_msgs::msg::OccupancyGrid& grid)` | 347 | **Bresenham line** rasterization checking occupancy grid cells (< 50 = free) |
| `nav_msgs::msg::OccupancyGrid generateOccupancyGrid()` | 375 | Extracts octree leaf nodes at a Z-thick slice, marks obstacles, applies circular inflation (`inflation_radius`) |
| `int main(int argc, char** argv)` | 432 | Spins with `MultiThreadedExecutor` |

---

## 4. `move.cpp` — `SwarmPlanner` (1497 lines)

**Purpose:** The **swarm mission manager**. The most complex node — assigns drones to waypoints, plans A* paths, manages batteries/charging, handles collision avoidance, returns all drones home.

### States: `TAKEOFF → MISSION → RETURN_TO_HOME`

### Key Sub-Structures

| Struct | Line | Purpose |
|---|---|---|
| `struct idx { int i, j; }` | 30 | Grid coordinate pair `{i, j}` |
| `struct DroneBatteryState { float percentage; bool has_charged; ... }` | 32 | Per-drone battery: percentage, charging flags, assigned slot, charge timer |
| `struct AStarNode { int idx_flat; int f_score; bool operator>(...) const; }` | 44 | Priority queue node: flat index + f_score (min-heap via `operator>`) |
| `enum class SwarmState { TAKEOFF, MISSION, RETURN_TO_HOME }` | 50 | State machine enum |
| `struct DroneJob { std::string id; bool is_assigned; ... }` | 923 | Job descriptor: drone ID, assigned flag, target position/z |
| `struct DroneCmd { std::string id; std::vector<...> waypoints; }` | 1378 | Batch command buffer: ID + waypoint list |

### Functions

| Signature | Line | What it does |
|---|---|---|
| `SwarmPlanner()` | 54 | Declares params; subscribes to `/drone/waypoint_array`, `/mission/z_target`, per-drone `/cf_X/pose`, `/AGV/pose`; creates OctoMap client, RTH/mission_done publishers; creates init timer (self-cancelling), planning timer (4 Hz), control timer (20 Hz); calls `initializeBMS()` |
| `void initializeBMS()` | 234 | Loads charging area from YAML config via utils; creates 2 charging slots (diagonally opposite at ±0.8m from center); subscribes to per-drone `/cf_X/battery_status`; creates land & takeoff service clients |
| `double dist2D(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b)` | 295 | Horizontal distance `hypot(dx, dy)` |
| `double dist3D(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b)` | 300 | 3D Euclidean distance |
| `void fetchOctomapOnce()` | 309 | Async call to OctoMap service; deserializes tree; calls `generateOccupancyGrid()`; bumps refcount into shared_ptr |
| `nav_msgs::msg::OccupancyGrid generateOccupancyGrid()` | 326 | Same as bfs.cpp: extracts Z-slice, inflates obstacles, returns grid |
| `bool hasGridLineOfSight(const idx& start, const idx& end, const nav_msgs::msg::OccupancyGrid& map)` | 386 | Bresenham with neighbor safety buffer check |
| `std::vector<idx> bfs(idx start, idx end, const nav_msgs::msg::OccupancyGrid& map)` | 433 | Actually **A\*** (naming mismatch). Uses `thread_local` visited-token trick to avoid reallocation; 8-connected, lazy skipping, priority queue with Chebyshev heuristic `max(\|dx\|, \|dy\|)` |
| `std::vector<geometry_msgs::msg::Point> runPlanningPipeline(const nav_msgs::msg::OccupancyGrid& map, geometry_msgs::msg::Point target, geometry_msgs::msg::Point current_pos, double target_z)` | 529 | Calls A*, then greedily jumps via LOS to reduce waypoints (no 3D first/last special case like bfs.cpp) |
| `bool canMatch(int u, double threshold, const std::vector<std::vector<double>>& costs, std::vector<int>& match, std::vector<bool>& vis)` | 579 | **DFS augmenting path** for bipartite matching (Hungarian-like); checks if drone `u` can match to a target within `threshold` |
| `void runSwarmSystem()` | 593 | **Main entry point @ 4 Hz:** gates on poses + map ready; dispatches to `handleTakeoff()`, `handleMission()`, or `handleReturnToHome()`; publishes RTH state on transitions only |
| `void handleBatteryLogic(const std::string& id)` | 621 | Per-drone battery FSM: (1) if charging and ≥88% → switch to `is_leaving_charger`; (2) if going-to-charge and within 0.1m of slot → switch to `is_charging` |
| `void handleTakeoff()` | 660 | Calls takeoff service for all drones, waits for all to reach z_target, transitions to MISSION |
| `void handleMission()` | 690 | **Core mission logic:** 1) AGV loop detection; 2) Snapshot grid/targets/poses; 3) BMS: classify available vs charging, trigger low-battery (≤70%) charging, smart recall (≥10% gain or no available drones); 4) Emergency RTH if any battery <25%; 5) **Bipartite matching** via binary search + DFS augmenting paths; 6) Build jobs: assigned → target, idle → shadow anchor, charging → slot approach/land; 7) A* planning; 8) Atomic commit to `active_commands_` |
| `void handleReturnToHome()` | 1137 | **Sequential return** (lowest battery first): drones before `rth_index_` stay landed, after hover, active navigates home via A*; increments index when z<0.15; at end → land service + `/mission_done = true` |
| `geometry_msgs::msg::Point applyRepulsion(geometry_msgs::msg::Point target, geometry_msgs::msg::Point current, const std::map<std::string, geometry_msgs::msg::Point>& all_positions, const std::map<std::string, geometry_msgs::msg::Point>& velocities, std::string my_id)` | 1301 | **Velocity-damped collision avoidance:** proportional push + derivative damping for neighbors within `safe_radius` (0.7m, 0.0 during RTH); clamps max shift to 0.5m |
| `void publishCommands()` | 1377 | **Control loop @ 20 Hz:** snapshots commands/poses, estimates velocity (finite difference), picks next waypoint, applies repulsion, clamps step to `max_speed * dt`, publishes to `/cf_X/cmd_position` |
| `int main(int argc, char** argv)` | 1486 | Spins with `MultiThreadedExecutor` |

---

## 5. `utils.cpp` / `utils.hpp` — Shared Utilities

| Signature | Line | What it does |
|---|---|---|
| `int get_num_robots()` | 6 | Returns `NUM_ROBOTS` env var (default: 5) |
| `std::string get_charging_file()` | 15 | Returns `CHARGING_FILE` env var (default: "5") |
| `double get_comm_range()` | 23 | Returns `COMM_RANGE` env var (default: 70) |
| `std::vector<double> charging_area_upper_left()` | 35 | Reads YAML config, returns upper-left corner of charging area |
| `std::vector<double> charging_area_down_right()` | 38 | Same YAML, returns lower-right corner |

---

## 6. `plot.py` — `plot_simulation_path()` (77 lines)

**Purpose:** Offline visualization of logged path CSVs. Reads `path_log.csv`, plots AGV trace, communication chain, drones, base station, AGV. Saves `simulation_plot.png`.

---

## Complete Data Flow

```
                     ┌──────────────────────┐
                     │   OctoMap Service     │
                     │  (octomap_binary)     │
                     └──────┬───────────────┘
                            │
              ┌─────────────┼─────────────────┐
              ▼             ▼                  ▼
   prediction.cpp   occupancy_grid.cpp    bfs.cpp / move.cpp
   (service→topic)  (2D slice @ Z)        (fetch on startup)
              │             │
              ▼             ▼
   /octomap_binary_topic   /map_slice
                            │
              ┌─────────────┘
              ▼
        bfs.cpp: runPlanningPipeline()
              │
              ├── generateOccupancyGrid()  (from OctoMap)
              ├── bfs()                    (BFS on grid: base → AGV)
              ├── LOS waypoint reduction
              ├── publishPoseArray()       → /drone/waypoint_array
              └── z_target_pub_            → /mission/z_target
                            │
                            ▼
                    move.cpp: SwarmPlanner
                            │
              ┌─────────────┼──────────────────┐
              ▼             ▼                   ▼
        handleTakeoff()  handleMission()   handleReturnToHome()
              │             │                   │
              │    ┌────────┴────────┐          │
              │    ▼                 ▼          │
              │  BMS Logic    Bipartite         │
              │  (battery      Matching         │
              │   check,       (canMatch +      │
              │   charge       binary search)   │
              │   trigger,                      │
              │   recall)     A* Planning       │
              │                 (bfs/A*)         │
              │    ▼                            │
              │  active_commands_               │
              │         │                       │
              └─────────┼───────────────────────┘
                        ▼
              publishCommands()  (20Hz)
                        │
                  applyRepulsion()
                  (velocity-damped collision avoidance)
                        │
                  Speed limiting
                  (max_speed * dt interpolation)
                        │
                        ▼
              /cf_X/cmd_position  (per drone)
```

