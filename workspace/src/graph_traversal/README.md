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

## 1. `prediction.cpp` — OctomapServiceToTopic (59 lines)

**Purpose:** Bridges the OctoMap *service* (`octomap_binary`) to a *topic* (`octomap_binary_topic`). Retries every 2s until the first successful call, then cancels the timer (one-shot).

| Function | Line | What it does |
|---|---|---|
| `timerCallback()` | 26 | Every 2s, checks service readiness, calls `async_send_request`; on success, publishes the binary map and **cancels the timer** |
| `main()` | 53 | Spins the node |

---

## 2. `occupancy_grid.cpp` — OctomapToPlane (131 lines)

**Purpose:** Extracts a **2D horizontal slice** from the OctoMap at a given Z height and publishes as `nav_msgs::msg::OccupancyGrid` on `/map_slice`. Saves `latest_plane_slice.png`. Retries every `retry_period` until the first successful call, then cancels the timer (one-shot).

| Function | Line | What it does |
|---|---|---|
| `timerCallback()` | 32 | Every `retry_period` seconds (default 2.0), calls OctoMap service |
| `handleResponse()` | 44 | Deserializes OctoMap → iterates leaf nodes in a Z-thick bounding box → marks occupied cells in a grid → publishes + saves PNG → **cancels the timer** |
| `main()` | 126 | Spins the node |

---

## 3. `bfs.cpp` — OctomapBFSPlanner (440 lines)

**Purpose:** The **relay-chain path planner**. Plans a chain of waypoints from base station (0,0) to the AGV using BFS (8-connected), then applies line-of-sight waypoint reduction. Publishes to `/drone/waypoint_array`.

| Function | Line | What it does |
|---|---|---|
| **Constructor** | 27 | Declares params (`z_target`, `inflation_radius`, `max_dist`, `frame_id`); creates AGV pose subscriber, waypoint publisher, z_target publisher, OctoMap service client, init timer (cancels itself after first map), main plan loop timer at ~3 Hz |
| `timerLoop()` | 105 | Main loop: captures AGV pose, detects AGV loops (return to start → raises altitude), calls `runPlanningPipeline()`, publishes waypoints and z_target |
| `fetchOctomapOnce()` | 158 | Calls OctoMap service, deserializes tree, computes `z_step_` from map `z_max` (divides into ~10 steps), generates initial occupancy grid, sets `map_ready_` |
| `publishPoseArray()` | 205 | Converts path points → `PoseArray` and publishes |
| `bfs()` | 219 | **BFS** (8-connected) on occupancy grid; returns vector of `idx` from start to end; early exit if start/end out of bounds |
| `dist3D()` | 260 | 3D Euclidean distance (static) |
| `runPlanningPipeline()` | 265 | Converts world → grid coords, calls `bfs()`, greedily reduces waypoints using LOS check (Bresenham) within `max_dist`; first/last segments use 3D distance (base at z=0, AGV on ground), intermediate use 2D; enforces `MIN_WAYPOINT_SEP = 2.0m` |
| `hasLineOfSight()` | 347 | **Bresenham line** rasterization checking occupancy grid cells (< 50 = free) |
| `generateOccupancyGrid()` | 375 | Extracts octree leaf nodes at a Z-thick slice, marks obstacles, applies circular inflation (`inflation_radius`) |
| `main()` | 432 | Spins with `MultiThreadedExecutor` |

---

## 4. `move.cpp` — SwarmPlanner (1497 lines)

**Purpose:** The **swarm mission manager**. The most complex node — assigns drones to waypoints, plans A* paths, manages batteries/charging, handles collision avoidance, returns all drones home.

### States: `TAKEOFF → MISSION → RETURN_TO_HOME`

### Key Sub-Structures

| Struct | Line | Purpose |
|---|---|---|
| `idx` | 30 | Grid coordinate pair `{i, j}` |
| `DroneBatteryState` | 32 | Per-drone battery: percentage, charging flags, assigned slot, charge timer |
| `AStarNode` | 44 | Priority queue node: flat index + f_score (min-heap via `operator>`) |
| `SwarmState` | 50 | Enum: `TAKEOFF`, `MISSION`, `RETURN_TO_HOME` |
| `DroneJob` | 923 | Job descriptor: drone ID, assigned flag, target position/z |
| `DroneCmd` | 1378 | Batch command buffer: ID + waypoint list |

### Functions

| Function | Line | What it does |
|---|---|---|
| **Constructor** | 54 | Declares params; subscribes to `/drone/waypoint_array`, `/mission/z_target`, per-drone `/cf_X/pose`, `/AGV/pose`; creates OctoMap client, RTH/mission_done publishers; creates init timer (self-cancelling), planning timer (4 Hz), control timer (20 Hz); calls `initializeBMS()` |
| `initializeBMS()` | 234 | Loads charging area from YAML config via utils; creates 2 charging slots (diagonally opposite at ±0.8m from center); subscribes to per-drone `/cf_X/battery_status`; creates land & takeoff service clients |
| `dist2D()` | 295 | Horizontal distance `hypot(dx, dy)` |
| `dist3D()` | 300 | 3D Euclidean distance |
| `fetchOctomapOnce()` | 309 | Async call to OctoMap service; deserializes tree; calls `generateOccupancyGrid()`; bumps refcount into shared_ptr |
| `generateOccupancyGrid()` | 326 | Same as bfs.cpp: extracts Z-slice, inflates obstacles, returns grid |
| `hasGridLineOfSight()` | 386 | Bresenham with neighbor safety buffer check |
| `bfs()` | 433 | Actually **A\*** (naming mismatch). Uses `thread_local` visited-token trick to avoid reallocation; 8-connected, lazy skipping, priority queue with Chebyshev heuristic `max(\|dx\|, \|dy\|)` |
| `runPlanningPipeline()` | 529 | Calls A*, then greedily jumps via LOS to reduce waypoints (no 3D first/last special case like bfs.cpp) |
| `canMatch()` | 579 | **DFS augmenting path** for bipartite matching (Hungarian-like); checks if drone `u` can match to a target within `threshold` |
| `runSwarmSystem()` | 593 | **Main entry point @ 4 Hz:** gates on poses + map ready; dispatches to `handleTakeoff()`, `handleMission()`, or `handleReturnToHome()`; publishes RTH state on transitions only |
| `handleBatteryLogic()` | 621 | Per-drone battery FSM: (1) if charging and ≥88% → switch to `is_leaving_charger`; (2) if going-to-charge and within 0.1m of slot → switch to `is_charging` |
| `handleTakeoff()` | 660 | Calls takeoff service for all drones, waits for all to reach z_target, transitions to MISSION |
| `handleMission()` | 690 | **Core mission logic:** 1) AGV loop detection; 2) Snapshot grid/targets/poses; 3) BMS: classify available vs charging, trigger low-battery (≤70%) charging, smart recall (≥10% gain or no available drones); 4) Emergency RTH if any battery <25%; 5) **Bipartite matching** via binary search + DFS augmenting paths; 6) Build jobs: assigned → target, idle → shadow anchor, charging → slot approach/land; 7) A* planning; 8) Atomic commit to `active_commands_` |
| `handleReturnToHome()` | 1137 | **Sequential return** (lowest battery first): drones before `rth_index_` stay landed, after hover, active navigates home via A*; increments index when z<0.15; at end → land service + `/mission_done = true` |
| `applyRepulsion()` | 1301 | **Velocity-damped collision avoidance:** proportional push + derivative damping for neighbors within `safe_radius` (0.7m, 0.0 during RTH); clamps max shift to 0.5m |
| `publishCommands()` | 1377 | **Control loop @ 20 Hz:** snapshots commands/poses, estimates velocity (finite difference), picks next waypoint, applies repulsion, clamps step to `max_speed * dt`, publishes to `/cf_X/cmd_position` |
| `main()` | 1486 | Spins with `MultiThreadedExecutor` |

---

## 5. `utils.cpp` / `utils.hpp` — Shared Utilities

| Function | Line | What it does |
|---|---|---|
| `get_num_robots()` | 6 | Returns `NUM_ROBOTS` env var (default: 5) |
| `get_charging_file()` | 15 | Returns `CHARGING_FILE` env var (default: "5") |
| `get_comm_range()` | 23 | Returns `COMM_RANGE` env var (default: 70) |
| `charging_area_upper_left()` | 35 | Reads YAML config, returns upper-left corner of charging area |
| `charging_area_down_right()` | 38 | Same YAML, returns lower-right corner |

---

## 6. `plot.py` — plot_simulation_path() (77 lines)

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

---

## Complete Function Table

| # | File | Function | Line | Scope | Purpose |
|---|---|---|---|---|---|
| 1 | prediction.cpp | `timerCallback()` | 26 | OctomapServiceToTopic | Retries every 2s; on success publishes topic once + cancels timer |
| 2 | prediction.cpp | `main()` | 53 | — | ROS spin |
| 3 | occupancy_grid.cpp | `timerCallback()` | 32 | OctomapToPlane | Triggers OctoMap fetch every `retry_period` |
| 4 | occupancy_grid.cpp | `handleResponse()` | 44 | OctomapToPlane | Deserializes OctoMap, builds 2D grid, publishes + saves PNG + cancels timer |
| 5 | occupancy_grid.cpp | `main()` | 126 | — | ROS spin |
| 6 | bfs.cpp | Constructor | 27 | OctomapBFSPlanner | Params, subs/pubs/clients, init & plan timers |
| 7 | bfs.cpp | `timerLoop()` | 105 | OctomapBFSPlanner | Main ~3Hz loop: AGV pose, loop detection, planning, publish |
| 8 | bfs.cpp | `fetchOctomapOnce()` | 158 | OctomapBFSPlanner | Fetches OctoMap, computes z_step, generates initial grid |
| 9 | bfs.cpp | `publishPoseArray()` | 205 | OctomapBFSPlanner | Converts path→PoseArray, publishes |
| 10 | bfs.cpp | `bfs()` | 219 | OctomapBFSPlanner | BFS 8-connected grid search |
| 11 | bfs.cpp | `dist3D()` | 260 | OctomapBFSPlanner | 3D Euclidean distance |
| 12 | bfs.cpp | `runPlanningPipeline()` | 265 | OctomapBFSPlanner | BFS + LOS reduction + min separation filter |
| 13 | bfs.cpp | `hasLineOfSight()` | 347 | OctomapBFSPlanner | Bresenham ray check on occupancy grid |
| 14 | bfs.cpp | `generateOccupancyGrid()` | 375 | OctomapBFSPlanner | Z-slice extraction + inflation |
| 15 | bfs.cpp | `main()` | 432 | — | MultiThreadedExecutor spin |
| 16 | move.cpp | Constructor | 54 | SwarmPlanner | Params, subs, pubs, timers, BMS init |
| 17 | move.cpp | `initializeBMS()` | 234 | SwarmPlanner | Charging area, 2 slots, battery subs, land/takeoff clients |
| 18 | move.cpp | `dist2D()` | 295 | SwarmPlanner | Horizontal distance |
| 19 | move.cpp | `dist3D()` | 300 | SwarmPlanner | 3D Euclidean distance |
| 20 | move.cpp | `fetchOctomapOnce()` | 309 | SwarmPlanner | Async OctoMap → generate grid → cache |
| 21 | move.cpp | `generateOccupancyGrid()` | 326 | SwarmPlanner | Z-slice + inflation |
| 22 | move.cpp | `hasGridLineOfSight()` | 386 | SwarmPlanner | Bresenham + neighbor safety buffer |
| 23 | move.cpp | `bfs()` | 433 | SwarmPlanner | A*: Chebyshev heuristic, thread_local visited token, 8-connected |
| 24 | move.cpp | `runPlanningPipeline()` | 529 | SwarmPlanner | A* + LOS jump reduction |
| 25 | move.cpp | `canMatch()` | 579 | SwarmPlanner | DFS augmenting path for bipartite matching |
| 26 | move.cpp | `runSwarmSystem()` | 593 | SwarmPlanner | 4Hz dispatch: TAKEOFF/MISSION/RTH + RTH state publish |
| 27 | move.cpp | `handleBatteryLogic()` | 621 | SwarmPlanner | Battery FSM: charging→leaving, going→charging |
| 28 | move.cpp | `handleTakeoff()` | 660 | SwarmPlanner | Takeoff service, wait at Z, transition to MISSION |
| 29 | move.cpp | `handleMission()` | 690 | SwarmPlanner | Full mission: BMS → matching → A* → commit |
| 30 | move.cpp | `handleReturnToHome()` | 1137 | SwarmPlanner | Sequential RTH, land service, /mission_done |
| 31 | move.cpp | `applyRepulsion()` | 1301 | SwarmPlanner | Velocity-damped collision avoidance |
| 32 | move.cpp | `publishCommands()` | 1377 | SwarmPlanner | 20Hz control: velocity, repulsion, speed limit, publish |
| 33 | move.cpp | `main()` | 1486 | — | MultiThreadedExecutor spin |
| 34 | utils.cpp | `get_num_robots()` | 6 | Global | Env `NUM_ROBOTS` (default 5) |
| 35 | utils.cpp | `get_charging_file()` | 15 | Global | Env `CHARGING_FILE` (default "5") |
| 36 | utils.cpp | `get_comm_range()` | 23 | Global | Env `COMM_RANGE` (default 70) |
| 37 | utils.cpp | `charging_area_upper_left()` | 35 | Global | YAML: charging area UL corner |
| 38 | utils.cpp | `charging_area_down_right()` | 38 | Global | YAML: charging area DR corner |
| 39 | plot.py | `plot_simulation_path()` | 6 | Global | Reads CSV, plots chain visualization, saves PNG |
