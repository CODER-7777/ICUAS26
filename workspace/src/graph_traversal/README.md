# graph_traversal Package

ROS 2 C++ package for drone relay-chain planning using OctoMap, BFS, A*, and swarm management. Designed for the ICUAS'26 competition.

## System Architecture

4 executables (nodes) working together:

```
prediction  ──► occupancy_grid  ──► bfs  ──► move
(OctoMap        (2D slice           (BFS chain    (Swarm mission
 service        at Z height)         planner)      manager;
 → topic)                                           5 submodules)
```

Plus `utils` (shared helpers), `plot.py` (offline visualization), and a standalone `SearchManager` (in `search_manager.hpp` / `search_manager.cpp`) for inactive drone ArUco search.

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

## 4. `SwarmPlanner` — Split across modular files

**Purpose:** The **swarm mission manager**. Assigns drones to waypoints, plans A* paths, manages batteries/charging, handles collision avoidance, returns all drones home. Originally monolithic (1497 lines), now split into 1 header + 6 implementation files + a thin `main()` entry point.

### Module Dependency & Data Flow

```
                          ┌──────────────────────────────────────────┐
                          │            types.hpp                     │
                           │  (idx, DroneBatteryState, AStarNode,     │
                           │   SwarmState, DroneRole, PoleInfo,       │
                           │   ViewZone)                               │
                          └──────────┬───────────────────────┬───────┘
                                     │                       │
                          ┌──────────▼──────────┐   ┌───────▼────────┐
                          │  search_manager.*   │   │  utils.*       │
                          │  (pole detection,   │   │  (config/env   │
                          │   zone generation,  │   │   helpers)      │
                          │   LOS anchoring)    │   └────────┬───────┘
                          └──────────┬──────────┘            │
                                     │                       │
                          ┌──────────▼───────────────────────▼───┐
                          │          swarm_planner.hpp            │
                          │  (class declaration, all members,     │
                          │   inline dist2D/dist3D helpers)       │
                          └──────────┬───────────────────────────┘
                                     │
                    ┌────────────────┼───────────────────┬──────────────────┐
                    ▼                ▼                   ▼                  ▼
        ┌───────────────────┐ ┌──────────────┐ ┌─────────────┐ ┌──────────────────┐
        │swarm_planner_init │ │swarm_planner_│ │swarm_planner│ │swarm_planner_rth │
        │ (constructor,     │ │grid          │ │_mission     │ │ (return-to-home, │
        │  BMS setup)       │ │(octomap, A*, │ │(state mach.,│ │  land service)   │
        │                   │ │ LOS, grid)   │ │ matching,   │ │                  │
        │                   │ │              │ │ job build)  │ │                  │
        └───────────────────┘ └──────────────┘ └─────────────┘ └──────────────────┘
                    │                │                │                  │
                    ▼                ▼                ▼                  ▼
        ┌─────────────────────────────────────────────────────────────────────┐
        │                      swarm_planner_control                          │
        │  (publishCommands @ 20 Hz: reprusion, speed limit, publish)        │
        └─────────────────────────────────────────────────────────────────────┘
```

### File Descriptions

#### `types.hpp` — Shared type definitions
Defines all data structures used across the swarm modules so every file sees the same types without circular includes:
- **`idx`** — grid coordinate `{i, j}` used by `bfs()`, `hasGridLineOfSight()`, and `runPlanningPipeline()` to index into the occupancy grid.
- **`DroneBatteryState`** — per-drone battery FSM state: `percentage`, `has_charged` (one-shot flag), `is_charging`/`is_going_to_charge`/`is_leaving_charger` (three-way charge state), `assigned_slot`, `start_charge_percentage`, and `start_leaving_time` for the clearance timeout.
- **`AStarNode`** — priority queue element for the A* planner: packs `idx_flat` + `f_score` with `operator>` for `std::priority_queue` min-heap.
- **`SwarmState`** — top-level state machine: `TAKEOFF → MISSION → RETURN_TO_HOME`.
- **`DroneRole`** — per-drone role enum: `SEARCH` (idle, assigned to pole viewing zones), `CHAIN_COMPONENT` (part of the relay chain between base station and AGV), `RTH` (returning to base), `LANDING` (executing landing descent). Roles are assigned every tick in `handleMission()` and `handleReturnToHome()`.
- **`PoleInfo`/`ViewZone`** — pole geometry and per-pole viewing zones used by `SearchManager` for ArUco search.

#### `swarm_planner.hpp` — Central class declaration
Declares the entire `SwarmPlanner` class inheriting `rclcpp::Node`:
- All private member variables (subscribers, publishers, clients, timers, state variables, battery maps, charging slots, `SearchManager` instance).
- `std::map<std::string, DroneRole> drone_roles_` — per-drone role tracker, updated every tick by `handleMission()` and `handleReturnToHome()` to reflect current drone assignment.
- Method declarations for every function (split across the 6 `.cpp` files).
- Inline `dist2D()` / `dist3D()` helpers — trivial enough that inlining avoids call overhead and keeps them visible to all translation units.

#### `swarm_planner_init.cpp` — Node construction and BMS setup
Handles everything that runs once at startup:

**`SwarmPlanner()` constructor:**
1. Declares ROS2 parameters: `inflation_radius`, `z_target`, `max_speed`.
2. Reads `NUM_ROBOTS` → builds drone ID list `cf_1`..`cf_N`.
3. Creates a **Reentrant** callback group so subscribers and timers can run concurrently in the `MultiThreadedExecutor`.
4. Sets up subscribers:
   - `/drone/waypoint_array` — BFS chain waypoints (updated at ~3 Hz).
   - `/mission/z_target` — altitude target from AGV loop detection (BFS side).
   - Per-drone `/cf_X/pose` — captures initial home position (patched with `pole_z_bottom_` when the octomap arrives).
   - `/AGV/pose` — AGV position for loop counting.
5. Creates service clients: OctoMap `GetOctomap`, per-drone `Land`/`Takeoff`.
6. Creates publishers: `RTH_STATE` (transient_local, only on transitions), `/mission_done`.
7. Creates `/drone_roles` publisher (`std_msgs::msg::String`, 10 Hz) — publishes comma-separated `id:ROLE` pairs every planning tick for monitoring and visualization.
8. Fires three timers:
   - **`init_timer_`** (500 ms, self-cancelling) — polls OctoMap until the first map arrives.
   - **`timer_`** (250 ms, 4 Hz) — `runSwarmSystem()`: the heavy planning tick.
   - **`control_timer_`** (50 ms, 20 Hz) — `publishCommands()`: lightweight repulsion + output.

**`initializeBMS()` (called at end of constructor):**
1. Reads charging area corners from YAML via `utils.hpp` → computes centre.
2. Creates 2 charging slots at ±0.8 m diagonally from centre (2.2 m separation > 1.4 m required).
3. Subscribes to per-drone `/cf_X/battery_status`.
4. Creates per-drone Land/Takeoff service clients.

#### `swarm_planner_grid.cpp` — OctoMap and path planning
All functions that touch the occupancy grid or compute paths:

**`fetchOctomapOnce()`:**
- Calls the OctoMap service asynchronously.
- On response: casts binary message → `OcTree`, generates the initial occupancy grid, bumps it into a `shared_ptr` for lock-free snapshotting, then calls `search_mgr_.buildFromOctomap(*tree_)` to detect poles and generate search zones.
- Patches any home positions already captured with the correct ground height (`pole_z_bottom_`).

**`generateOccupancyGrid()`:**
- Slices the octree at `z_target` ± 1 cell thickness.
- Marks occupied leaf cells as obstacles.
- Applies circular inflation (`inflation_radius`) so A* paths keep clearance from walls.

**`hasGridLineOfSight()`:**
- Bresenham line rasterization between two grid cells.
- Checks every cell (and its 4 neighbours) is free (<50 occupancy) — this buffer prevents the drone from grazing walls.

**`bfs()` (actually A* with Chebyshev heuristic):**
- Uses `thread_local` vectors + a generation-token trick (`visited_token[]`) to avoid heap allocation on every replan cycle — the vectors grow to max size once and are reused.
- 8-connected expansion, lazy-skipping optimization for revisited nodes.
- Priority queue with `f_score = g_score + h_score` where `h_score = max(|dx|, |dy|)`.

**`runPlanningPipeline()`:**
- Converts world coordinates → grid indices, calls `bfs()`.
- Greedy waypoint reduction: for each path index, jumps to the farthest reachable cell via `hasGridLineOfSight()`, producing a sparse waypoint list.

#### `swarm_planner_mission.cpp` — State machine and mission logic
The orchestration layer — decides what each drone does every planning tick (4 Hz):

**`canMatch()`:**
- DFS-based augmenting path search (Hungarian algorithm style).
- Used in binary search over distance thresholds to find the minimum-cost bipartite matching between available drones and mission targets.

**`runSwarmSystem()`:**
- Top-level dispatcher gated on `swarm_poses_.size() >= N` and `map_ready_`.
- Routes to `handleTakeoff()`, `handleMission()`, or `handleReturnToHome()` based on `current_state_`.
- Publishes `RTH_STATE` **only on transitions** (avoids waking the ArUco node unnecessarily).

**`handleTakeoff()`:**
- First call: sends async takeoff requests to all drones (height 1.0 m, 2 s duration).
- Subsequent calls: waits until every drone's pose Z reaches `z_target` within `REACH_THRESHOLD` (0.15 m) **and** at least one waypoint array has arrived → transitions to `MISSION`.

**`handleMission()` — the core (largest function):**
1. **AGV loop detection** — measures distance from AGV start; when AGV returns to start after moving away, increments loop counter and raises `z_target` by `HEIGHT_BOOST` (1.0 m).
2. **Snapshot** — atomically copies grid (shared_ptr refcount), targets, and poses under the mutex.
3. **Search marking** — calls `search_mgr_.markVisited()` for every drone's current position.
4. **BMS classification** — iterates all drones: those already charging/going/leaving go to the `charging_drones` list; the rest check battery ≤ 70% → if a slot is free and concurrency limit (2) isn't reached, the drone is sent to charge. Remaining drones go to `available_drones`.
5. **Smart recall** — if `available_drones.size() < num_targets`, recalls the highest-battery charging drone(s) if they've gained ≥10% charge (or if there are zero available).
6. **Emergency check** — any battery under 25% triggers immediate `RETURN_TO_HOME`.
7. **Role initialization** — all drones set to `SEARCH`; overridden per assignment below.
8. **Bipartite matching** — builds a cost matrix (3D distance), binary-searches over sorted distances to find the minimum threshold that achieves a full match via `canMatch()`.
9. **Job building + role assignment** — for available drones: matched ones get `CHAIN_COMPONENT` + mission target; unmatched (idle) ones stay `SEARCH` and get a search zone via `search_mgr_.pickNextZone()` or failover to a shadow anchor. For charging drones: `RTH` (en route) or `LANDING` (on slot) + clearance maneuver, slot approach, or hold-at-slot depending on substate.
10. **A* planning** — calls `runPlanningPipeline()` for every drone job, builds command waypoint lists with yaw overrides for search zones.
11. **Atomic commit** — swaps `active_commands_` under the mutex so the 20 Hz control loop sees a consistent state.

#### `swarm_planner_bms.cpp` — Battery charging FSM
A single function `handleBatteryLogic(id)` called per-drone per tick:

1. **If `is_charging` and percentage ≥ 88%** → switch to `is_leaving_charger`, record `start_leaving_time`, keep the slot occupied (released only after the clearance maneuver finishes).
2. **If `is_going_to_charge` and within 0.1 m of the assigned slot** → switch to `is_charging`, set Z target to 0.05 m (landed on slot).
3. Otherwise no state change — the drone continues its current trajectory.

Slot management is handled in `handleMission()` (assignment, recall, late release).

#### `swarm_planner_rth.cpp` — Sequential return to home
Handles the `RETURN_TO_HOME` state. Assigns roles each tick:

1. **First call only** — sorts all drones by battery percentage (ascending: lowest first) into `rth_sorted_ids_`.
2. **Sequential dispatch** — `rth_index_` tracks which drone is actively returning:
   - Drones **before** `rth_index_` → already landed, role `LANDING`, commanded to `z = 0.05` at home.
   - Drones **after** `rth_index_` → waiting, role `RTH`, hover in place.
   - Drone at `rth_index_` → role `RTH` (navigating) or `LANDING` (arrived), navigates home via A*; when within 0.2 m (2D) and Z < 0.15 m, increments `rth_index_`.
3. **Final landing** — when `rth_index_ >= N`: calls the Land service for all drones, then publishes `/mission_done = true` (one-shot).

#### `swarm_planner_control.cpp` — Real-time control output
Runs at 20 Hz (every 50 ms) on a separate timer:

**`publishCommands()`:**
1. Snapshots `active_commands_` and current poses under the mutex.
2. For each drone: picks the **next waypoint** (index 1 if more than 1 waypoint, else index 0).
3. Calls `applyRepulsion()` to nudge the target away from nearby drones.
4. **Speed limiting** — computes the Euclidean step needed; if it exceeds `max_speed * 0.05 s`, interpolates toward the target so the drone never exceeds the speed limit in a single control interval.
5. Publishes the adjusted command to `/cf_X/cmd_position`.

**`applyRepulsion()`:**
- Iterates all other drones; if within `safe_radius` (0.7 m normal, 0.0 m during RTH), applies a repulsive force inversely proportional to distance.
- Clamps the total shift to `MAX_REPULSION_STEP` (1.0 m) per tick to prevent jerky motion.

#### `move.cpp` — Entry point
Minimal: calls `rclcpp::init`, creates a `SwarmPlanner` node, spins on a `MultiThreadedExecutor` (critical for allowing subscribers to interleave with the planning timer), then shuts down.

### States: `TAKEOFF → MISSION → RETURN_TO_HOME`

### Key Sub-Structures

| Struct | File | Purpose |
|---|---|---|
| `struct idx { int i, j; }` | `types.hpp` | Grid coordinate pair `{i, j}` |
| `struct DroneBatteryState { float percentage; bool has_charged; ... }` | `types.hpp` | Per-drone battery: percentage, charging flags, assigned slot, charge timer |
| `struct AStarNode { int idx_flat; int f_score; bool operator>(...) const; }` | `types.hpp` | Priority queue node: flat index + f_score (min-heap via `operator>`) |
| `enum class SwarmState { TAKEOFF, MISSION, RETURN_TO_HOME }` | `types.hpp` | State machine enum |
| `enum class DroneRole { RTH, LANDING, CHAIN_COMPONENT, SEARCH }` | `types.hpp` | Per-drone role: chain component, search, RTH, or landing. Assigned every tick by `handleMission()` / `handleReturnToHome()` |
| `struct DroneJob { std::string id; bool is_assigned; ... }` | `swarm_planner_mission.cpp` | Job descriptor: drone ID, assigned flag, target position/z |
| `struct DroneCmd { std::string id; std::vector<...> waypoints; }` | `swarm_planner_control.cpp` | Batch command buffer: ID + waypoint list |

### Functions

| Signature | File | What it does |
|---|---|---|
| `SwarmPlanner()` | `swarm_planner_init.cpp` | Declares params; subscribes to `/drone/waypoint_array`, `/mission/z_target`, per-drone `/cf_X/pose`, `/AGV/pose`; creates OctoMap client, RTH/mission_done publishers; creates init timer (self-cancelling), planning timer (4 Hz), control timer (20 Hz); calls `initializeBMS()` |
| `void initializeBMS()` | `swarm_planner_init.cpp` | Loads charging area from YAML config via utils; creates 2 charging slots (diagonally opposite at ±0.8m from center); subscribes to per-drone `/cf_X/battery_status`; creates land & takeoff service clients |
| `double dist2D(...)` / `double dist3D(...)` | `swarm_planner.hpp` (inline) | Horizontal `hypot(dx, dy)` / 3D Euclidean distance |
| `void fetchOctomapOnce()` | `swarm_planner_grid.cpp` | Async call to OctoMap service; deserializes tree; calls `generateOccupancyGrid()`; bumps refcount into shared_ptr |
| `nav_msgs::msg::OccupancyGrid generateOccupancyGrid()` | `swarm_planner_grid.cpp` | Same as bfs.cpp: extracts Z-slice, inflates obstacles, returns grid |
| `bool hasGridLineOfSight(...)` | `swarm_planner_grid.cpp` | Bresenham with neighbor safety buffer check |
| `std::vector<idx> bfs(...)` | `swarm_planner_grid.cpp` | Actually **A\*** (naming mismatch). Uses `thread_local` visited-token trick to avoid reallocation; 8-connected, lazy skipping, priority queue with Chebyshev heuristic `max(\|dx\|, \|dy\|)` |
| `std::vector<...> runPlanningPipeline(...)` | `swarm_planner_grid.cpp` | Calls A*, then greedily jumps via LOS to reduce waypoints (no 3D first/last special case like bfs.cpp) |
| `bool canMatch(...)` | `swarm_planner_mission.cpp` | **DFS augmenting path** for bipartite matching (Hungarian-like); checks if drone `u` can match to a target within `threshold` |
| `void runSwarmSystem()` | `swarm_planner_mission.cpp` | **Main entry point @ 4 Hz:** gates on poses + map ready; dispatches to `handleTakeoff()`, `handleMission()`, or `handleReturnToHome()`; publishes RTH state on transitions only; publishes `/drone_roles` topic with current role of every drone |
| `void handleBatteryLogic(...)` | `swarm_planner_bms.cpp` | Per-drone battery FSM: (1) if charging and ≥88% → switch to `is_leaving_charger`; (2) if going-to-charge and within 0.1m of slot → switch to `is_charging` |
| `void handleTakeoff()` | `swarm_planner_mission.cpp` | Calls takeoff service for all drones, waits for all to reach z_target, transitions to MISSION |
| `void handleMission()` | `swarm_planner_mission.cpp` | **Core mission logic:** 1) AGV loop detection; 2) Snapshot grid/targets/poses; 3) BMS: classify available vs charging, trigger low-battery (≤70%) charging, smart recall (≥10% gain or no available drones); 4) Emergency RTH if any battery <25%; 5) **Role init** → all drones default to `SEARCH`; 6) **Bipartite matching** via binary search + DFS augmenting paths → matched drones get `CHAIN_COMPONENT`; 7) Build jobs: `CHAIN_COMPONENT` → target, `SEARCH` → search zone/shadow anchor, `RTH`/`LANDING` → charge slot/clearance; 8) A* planning; 9) Atomic commit to `active_commands_` |
| `void handleReturnToHome()` | `swarm_planner_rth.cpp` | **Sequential return** (lowest battery first): drones before `rth_index_` stay landed (`LANDING`), after hover (`RTH`), active navigates home (`RTH` → `LANDING`); increments index when z<0.15; at end → land service + `/mission_done = true` |
| `geometry_msgs::msg::Point applyRepulsion(...)` | `swarm_planner_control.cpp` | **Velocity-damped collision avoidance:** proportional push + derivative damping for neighbors within `safe_radius` (0.7m, 0.0 during RTH); clamps max shift to 0.5m |
| `void publishCommands()` | `swarm_planner_control.cpp` | **Control loop @ 20 Hz:** snapshots commands/poses, estimates velocity (finite difference), picks next waypoint, applies repulsion, clamps step to `max_speed * dt`, publishes to `/cf_X/cmd_position` |
| `const char* roleToString(DroneRole r)` | `swarm_planner_mission.cpp` | Converts `DroneRole` enum to human-readable string (`SEARCH`, `CHAIN_COMPONENT`, `RTH`, `LANDING`) |
| `void publishRoles()` | `swarm_planner_mission.cpp` | Builds a comma-separated `id:ROLE` string for all drones and publishes to `/drone_roles` |
| `int main(int argc, char** argv)` | `move.cpp` | Spins with `MultiThreadedExecutor` |

---

## 5. `SearchManager` — Pole-based ArUco Search (`search_manager.hpp` / `search_manager.cpp`)

**Purpose:** Converts inactive (non-chain) drones into active ArUco seekers. Instead of idly shadowing the nearest chain anchor, each drone with role `SEARCH` inspects per-pole viewing zones it hasn't yet visited. Drones assigned to the relay chain (`CHAIN_COMPONENT`), returning to base (`RTH`), or landing (`LANDING`) are excluded from search task assignment.

### Key Types

| Struct/Class | File | Purpose |
|---|---|---|
| `struct PoleInfo { double cx, cy, radius, z_bottom, z_top; }` | `types.hpp` | Detected pole centre, radius, and vertical extent |
| `struct ViewZone { double x, y, z, yaw; int pole_idx, dir_idx, height_idx; bool visited; }` | `types.hpp` | A drone target position (with yaw facing the pole centre) for one cardinal direction × one height level |
| `class SearchManager` | `search_manager.hpp` | Detects poles from the octomap floor slab, generates 8 zones per pole (4 cardinal × 2 heights), tracks visited zones, picks nearest unvisited zone with LOS to an anchor |

### Functions

| Signature | What it does |
|---|---|
| `void buildFromOctomap(const octomap::OcTree& tree)` | Scans a floor slab (0.3m thick, 0.2m above ground) in the octree; flood-fills connected occupied cells; rejects components that are too small (noise) or too large (walls/fences); computes centroid → `PoleInfo`; generates 8 `ViewZone`s per pole. |
| `void markVisited(double x, double y, double z)` | Marks all zones within `visit_radius` (3D) as visited if the drone is on the correct (pole-facing) side of the pole. Called per drone per tick at the top of `handleMission()`. |
| `int pickNextZone(double drone_x, double drone_y, double drone_z, const std::vector<...>& anchors, double comm_range, const std::function<bool(...)>& losCheck)` | Returns the index of the nearest unvisited zone that has line-of-sight to at least one anchor (drone or base station). Returns -1 if no valid zone exists. |
| `bool allVisited() const` | Returns true iff all zones have been visited. |

### Data Flow in `handleMission()` (role-aware)

1. Snapshot grid, targets, poses (unchanged).
2. `searchMgr_.markVisited(...)` for every drone (any drone can mark a zone visited).
3. BMS classification + smart recall (unchanged).
4. **Role initialization** — all drones default to `SEARCH`.
5. Bipartite matching (unchanged) → matched drones get `CHAIN_COMPONENT` role.
6. Idle (unmatched) drones stay `SEARCH`: `searchMgr_.pickNextZone()` with LOS callback checking the 2D grid.
7. Charging drones → `RTH` (en route) or `LANDING` (on slot).
8. Build `jobs[]`: `CHAIN_COMPONENT` → target; `SEARCH` → zone target (fallback shadow anchor); `RTH`/`LANDING` → charge slot/clearance.
9. Override final waypoint yaw to face the pole when `has_search_yaw`.
10. A* planning + atomic commit (unchanged).

### ROS Parameters

| Param | Default | Description |
|---|---|---|
| `standoff_` | 1.5 m | Distance from pole surface to zone centre |
| `visit_radius_` | 1.5 m | Drone must be within this 3D distance to mark a zone visited |
| `min_pole_cells_` | 4 cells | Floor area threshold to count as a pole (filters noise) |
| `max_pole_extent_` | 1.5 m | Rejects components wider than this (walls/fences) |

---

## 6. `utils.cpp` / `utils.hpp` — Shared Utilities

| Signature | Line | What it does |
|---|---|---|
| `int get_num_robots()` | 6 | Returns `NUM_ROBOTS` env var (default: 5) |
| `std::string get_charging_file()` | 15 | Returns `CHARGING_FILE` env var (default: "5") |
| `double get_comm_range()` | 23 | Returns `COMM_RANGE` env var (default: 70) |
| `std::vector<double> charging_area_upper_left()` | 35 | Reads YAML config, returns upper-left corner of charging area |
| `std::vector<double> charging_area_down_right()` | 38 | Same YAML, returns lower-right corner |

---

## 7. `plot.py` — `plot_simulation_path()` (77 lines)

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
    prediction.cpp   occupancy_grid.cpp    bfs.cpp / move
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
                      SwarmPlanner
              (swarm_planner.hpp + 6 .cpp files)
                              │
                ┌─────────────┼──────────────────┐
                ▼             ▼                   ▼
          handleTakeoff()  handleMission()   handleReturnToHome()
       (swarm_planner_    (swarm_planner_    (swarm_planner_
        mission.cpp)       mission.cpp)       rth.cpp)
                │             │                   │
                │    ┌────────┴────────┐          │
                │    ▼                 ▼          │
                │  markVisited   Bipartite        │
                │  (SearchMgr)    Matching        │
                │    │            (canMatch +     │
                │    ▼            binary search)  │
                │  BMS Logic      │               │
                │  (battery    pickNextZone       │
                │   check,    (SearchMgr)         │
                │   charge        │               │
                │   trigger,      │               │
                │   recall)       │               │
                │ (swarm_planner_bms.cpp)         │
                │    │            │               │
                │    └─────┬──────┘               │
                │          ▼                      │
                │     A* Planning                 │
                │     (swarm_planner_grid.cpp)    │
                │          │                      │
                │  yaw override for search        │
                │          ▼                      │
                │  active_commands_               │
                │         │                       │
                └─────────┼───────────────────────┘
                          ▼
                publishCommands()  (20Hz)
              (swarm_planner_control.cpp)
                         │
                   applyRepulsion()
                   (collision avoidance)
                         │
                   Speed limiting
                   (max_speed * dt interpolation)
                         │
                         ▼
               /cf_X/cmd_position  (per drone)
```

