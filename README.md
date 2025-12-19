
---

## first_run.sh (root)

- Purpose: creates and runs the Docker container used for simulation and development (`crazysim_icuas_cont`), mounting the repository and forwarding X11 and the SSH agent. The container includes ROS2, the simulator, and packages in this repo.
- Typical use:
  1. Build the Docker image:
     ```bash
     docker build --ssh default -t crazysim_icuas_img .
     ```
  2. Run the container for the first time (this script creates the container and drops you into an interactive shell inside it):
     ```bash
     ./first_run.sh
     ```
  3. Re-enter a created container later with:
     ```bash
     docker start -i crazysim_icuas_cont
     # or
     docker exec -it crazysim_icuas_cont bash
     ```

Notes:
- The script prepares an Xauthority file for GUI forwarding and mounts the workspace into the container (`/root/ros2_ws/src/solution/` and `/root/path_planning/`).
- If you get Xauthority errors, try removing `/tmp/.docker.xauth` or running the script as root.

### Volume mounts used by `first_run.sh`

`first_run.sh` exposes several host paths into the container so the container can access displays, devices, the SSH agent and the repository. The important mounts are:

- `/tmp/.X11-unix:/tmp/.X11-unix:rw` — X11 socket for GUI forwarding (required if you want to run graphical apps like Gazebo or RViz from inside the container).
- `/dev:/dev` — exposes host devices (serial, USB) to the container so simulated or real hardware interfaces can be used.
- `/var/run/dbus/:/var/run/dbus/:z` — DBus socket; useful for desktop integration and some system services.
- `~/.ssh/ssh_auth_sock:/ssh-agent` and `--env SSH_AUTH_SOCK=/ssh-agent` — forwards your SSH agent into the container so git and SSH operations can use your keys without copying them.
- `--volume="$REPO_ROOT/solution/:/root/ros2_ws/src/solution/"` — mounts the host `solution/` directory into the container's ROS2 workspace source folder so changes on the host are immediately visible inside the container.
- `--volume="$REPO_ROOT/path_planning/:/root/path_planning/"` — mounts host `path_planning/` into the container for building/running the planner from inside the container.

Why these mounts matter:
- They allow running GUI tools, using GPU and device hardware, and live-editing code on the host while the container runs ROS and the simulator.
- Because the repo is mounted, you don't need to rebuild the Docker image to test code changes; edit on the host and run inside the container.

Troubleshooting and tips:
- If GUI apps fail to show, ensure you ran `xhost +local:docker` on the host and that `/tmp/.X11-unix` permissions are correct.
- If SSH agent forwarding doesn't work, check `echo $SSH_AUTH_SOCK` on host and that `~/.ssh/ssh_auth_sock` exists (the script symlinks it).
- When files created in the container appear owned by root on the host, you can either `chown` them from the host or start the container with a matching UID/GID mapping. For short experiments `sudo` inside the host may be simpler.
- To change what host folder is mounted into the ROS workspace, edit the `--volume="$REPO_ROOT/…"` lines in `first_run.sh` (for example to mount a different package or mount the whole repo into `/root/ros2_ws/src/`).

If you'd like, I can also update `first_run.sh` to (optionally) accept environment variables to override these mounts (safer for multi-user systems) — tell me if you want that and I will add it.

---

## path_planning/

Purpose: generate a tour across all waypoints and provide coordinates for the path follower.

Key files and behavior:
- `main.cpp` — reads `icuas_waypoints.json`, constructs per-building graphs, generates intra-graph edges, finds inter-graph connecting bridges, runs a solver (Chinese Postman / tour) and writes two artifacts:
  - `final_path.txt` — ordered waypoint ids for the tour.
  - `final_path_coordinates.csv` — ordered coordinates (CSV header `x,y,z`) used by the follower node.
- `graph_solver.hpp/cpp` — supporting solver algorithms.
- `path_planning_ros.py` — a ROS2 Python node which:
  - loads `final_path_coordinates.csv` (expects header x,y,z),
  - connects to Crazyflie services (`Takeoff`, `GoTo`, `Land`),
  - requests takeoff, sequentially sends waypoints using `/cf_1/go_to`, waits, and finally calls land.

Run example (host or inside container):
1. Build the planner (CMake):
   ```bash
   cd path_planning
   mkdir -p build && cd build
   cmake .. && make
   # binary will be `main` in this build dir
   ./main
   ```
2. Confirm `final_path_coordinates.csv` exists (in the `path_planning` folder). To test follower:
   - Start simulator (inside container) using `startup/start.sh`.
   - Run the follower node inside the ROS2 environment (inside container; ensure ROS2 sourced):
     ```bash
     python3 ../path_planning/path_planning_ros.py
     ```
   The script expects `final_path_coordinates.csv` in its working directory.

Notes:
- The Python follower uses time.sleep between waypoints and service-based synchronous calls — adjust durations in `path_planning_ros.py` to match your simulation timing.

---

## solution/

Purpose: mission logic, ROS nodes, and interfaces for multi-agent coordination and role assignment.

Layout (high-level):
- `aruco_mission/` — ArUco marker recognition nodes and mission glue.
- `octomap_map_generator/` — tools to create octomap files from 3D meshes (C++/CMake).
- `solution/` (Python package) — mission controllers and drone role implementations. Notable files:
  - `solution/solution/solution/center_role.py` — example controller that assigns the center role to a drone, monitors battery levels, swaps center drone when battery falls below threshold, and issues takeoff/goto/land commands.
  - `drone_role.py`, `tagger_drone.py` — other role controllers showing typical message/service usage.
- `solution_interfaces/` — ROS service types used across the system (e.g., `Assign` service).

How to run:
- The packages are intended to run inside the prepared Docker container. Use `startup/start.sh` to bring up the simulator and typical launch files. The `start.sh` can spawn multiple Crazyflies and the necessary services/topics used by the controllers.
- Mission controllers subscribe to `/drone_roles` and monitor `/cf_X/battery_status`. They use services like `/{cf}/takeoff`, `/{cf}/go_to`, and `/{cf}/land` to command drones.

