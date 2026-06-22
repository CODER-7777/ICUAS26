# ICUAS 2026: Autonomous Multi-UAV Swarm System

**ICUAS 2026** Swarm System is designed to coordinate a fleet of 5 Crazyflie nano-drones in an urban cityscape. The drones cooperatively navigate to locate ArUco markers on obstacle pillars while maintaining a communication relay chain to a moving ground vehicle (AGV). It integrates two-tier path planning, dynamic communication relays, and an event-driven battery management system.

<p align="center">
  <img src="https://via.placeholder.com/800x450?text=Insert+Demo+GIF+Here" width="800" height="450"/>
</p>

To run this project in minutes, check [Quick Start](#1-quick-start). Check other sections for more detailed information.

Please kindly star :star: this project if it helps you. We take great efforts to develop and maintain it :grin::grin:.

## Table of Contents
* [Quick Start](#1-quick-start)
* [Algorithms and Key Modules](#2-algorithms-and-key-modules)
* [Use in Your Application](#3-use-in-your-application)
* [Parameters](#4-parameters)

## 1. Quick Start
This project has been tested on Ubuntu 22.04 (ROS Humble).

Firstly, you should ensure you have **Docker** and the **NVIDIA Container Toolkit** installed.

Then, clone the repository and build the Docker image:
```bash
git clone https://github.com/CODER-7777/ICUAS26.git
cd ICUAS26
export DOCKER_BUILDKIT=1
docker build --ssh default -t crazysim_icuas_img .
```

Run the container for the first time:
```bash
./first_run.sh
```
*(This will create the docker container `crazysim_icuas_cont` and place you inside it).*

For subsequent use of the container, you can start and attach to it using:
```bash
docker start -i crazysim_icuas_cont
```

### Running the Simulation
Once inside the container, navigate to the startup directory (you can use the built-in alias):
```bash
cd_icuas26_competition
cd startup
```

Start the simulation (Tmuxinator will launch the Gazebo world and all Swarm Nodes):
```bash
./start.sh
```
*(To run the full competition city instead of the empty world, edit `_setup.sh` and set `export KEYWORD=icuas26_1` before running `start.sh`)*.

## 2. Algorithms and Key Modules

All planning algorithms along with other key modules are implemented in this repository:

- **Swarm Architecture**: Developed an **event-driven Finite State Machine (FSM)** for priority-based battery management and scheduling. The C++ ROS 2 node orchestrates 5 drones through `TAKEOFF` → `MISSION` → `RETURN-TO-HOME` phases, with real-time role assignment.

- **Two-Tier Path Planning**: Designed a two-tier path planning framework integrating **BFS** with **Bresenham line-of-sight optimization** for global route generation, and **A*** for local navigation to track a moving target with uncertain position estimates. Each drone plans in priority order and inflates its smoothed path into a reservation grid, guaranteeing collision-free trajectories.

- **Communication-Constrained Relay Chain**: Solves a connectivity-constrained multi-robot problem where drones must maintain a line-of-sight (LOS) relay chain between a base station and a moving AGV. Utilized **bipartite matching** for drone-to-waypoint assignment, achieving an optimized **O(log D · VE)** execution time to minimize the bottleneck edge length across the fleet.

- **Autonomous ArUco Search**: A `SearchManager` module extracts vertical obstacle pillars from OctoMap data, generates 8 viewpoint zones per pillar (4 cardinal directions × 2 heights), and assigns idle drones to unvisited zones using cooperative selection with drone-clearance and pillar-clearance constraints.

- **Battery Management System (BMS)**: Features a hot-swap charging strategy with concurrent slot management (max 2 drones charging simultaneously). Includes smart recall logic to only recall a charging drone if it has gained ≥10% charge or all other drones are unavailable, plus an emergency RTH triggered below 25% battery.

The full pipeline is validated in simulation (Gazebo Garden), achieving **Global Rank 6** and making us the **only Indian team** to qualify for the final round in Greece. We achieved detection of **7/8 ArUco markers** and sustained over **70% communication connectivity**.

## 3. Use in Your Application
If you want to use the ICUAS swarm components in your own application, please explore the `workspace/src/graph_traversal/` directory for the core C++ planning logic (A*, BFS, SearchManager). 

For the advanced control prototypes, check the `workspace/` directory for:
- **RL-Based Pursuit:** A PPO-based reinforcement learning node (`deploy_node.py`) using Stable Baselines3 for single-drone obstacle-avoidant tracking.
- **Distributed MPC:** A Model Predictive Control node (`distributed_mpc_node.py`) using CVXOPT for QP-based obstacle-avoidant trajectory optimization.

## 4. Parameters
The ROS implementation exposes several parameters inside `workspace/config.yaml`:

|Parameter|Definition|Default|
|---|---|---|
|`resolution`|Resolution of the simulation environment grid.|0.1 m|
|`max_steps`|Maximum steps for the simulation environment.|1000|
|`max_accel`|Maximum acceleration for the drones.|1.0 m/s^2|
|`n_rays`|Number of LIDAR-like rays used for observation in RL tracking.|12|
|`max_range`|Maximum range of the simulated sensors.|5.0 m|
|`max_comm_range`|Maximum communication range between drones to maintain the relay chain.|3.0 m|
|`goal_speed`|Target speed for the moving ground vehicle (AGV).|1.0 m/s|
