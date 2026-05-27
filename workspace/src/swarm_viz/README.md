# swarm_viz

Debug visualization for the `graph_traversal` swarm planner.

## What it shows (M1 + M2)

| Topic | Type | Purpose |
| --- | --- | --- |
| `/swarm_viz/drones` | `MarkerArray` | Labeled spheres for every drone + the base anchor |
| `/swarm_viz/trail/<drone_id>` | `nav_msgs/Path` | Recent pose trail per drone |
| `/swarm_viz/comm_graph` | `MarkerArray` (LINE_LIST) | Drone-drone and drone-base edges. **Green** = within `COMM_RANGE` and grid LOS clear; **Red** = within range but LOS blocked; **Yellow** (thin) = out of range |
| `/swarm_viz/comm_range_spheres` | `MarkerArray` | Optional translucent comm-range spheres (off by default) |

## What it consumes from the planner

The planner now publishes (latched, `transient_local`):

- `/swarm_viz/occupancy_slice` — the exact `nav_msgs/OccupancyGrid` used by BFS/LOS,
  so the visualization checks identical geometry.
- `/swarm_viz/base_anchor` — the charging-station centerpoint used as a comm anchor.

The LOS check itself lives in `graph_traversal/include/graph_traversal/los.hpp`
and is called by both the planner and this node, guaranteeing parity.

## Environment

Reads the same env vars the planner does:

- `NUM_ROBOTS` (default `5`) — drones `cf_1 .. cf_N`
- `COMM_RANGE` (default `70`)

## Run

```bash
# Build (from your colcon workspace root)
colcon build --packages-select graph_traversal swarm_viz
source install/setup.bash

# Standalone (assumes planner is already running)
ros2 launch swarm_viz swarm_viz.launch.py

# With Foxglove bridge on :8765
ros2 launch swarm_viz foxglove.launch.py
```

In Foxglove, open the 3D panel and add the topics above. Set the
"display frame" to `world` to match the planner.

## Debugging LOS failures

When you see a red edge in `/swarm_viz/comm_graph`:

1. Add a `Map` panel for `/swarm_viz/occupancy_slice` — that's the exact grid
   the planner sees at the current target altitude.
2. Trace the red line over the grid — the LOS check fails if **any** cell on
   the Bresenham line (or its 4-neighbors) is `>= 50` or unknown (`-1`).
3. Note the LOS check is **2D** (in the XY slice). A drone at a different Z
   from the grid's snapshot Z may have a 3D-clear path that this check still
   rejects — that's a known limitation and a candidate root cause to inspect.
