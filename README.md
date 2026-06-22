# ICUAS 2026: Autonomous Multi-UAV Swarm System

**ICUAS 2026** Swarm System is designed to coordinate a fleet of 5 Crazyflie nano-drones in an urban cityscape. The drones cooperatively navigate to locate ArUco markers on obstacle pillars while maintaining a communication relay chain to a moving ground vehicle (AGV). It integrates two-tier path planning, dynamic communication relays, and an event-driven battery management system.

<p align="center">
  <img src="https://via.placeholder.com/400x225?text=Swarm+Demo+1" width="400" height="225"/>
  <img src="https://via.placeholder.com/400x225?text=Swarm+Demo+2" width="400" height="225"/>
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

Firstly, you should install the following required libraries:
Docker Engine & Docker Compose, NVIDIA GPU with Container Toolkit (recommended for Gazebo).

Then simply clone and compile our package:
```bash
git clone https://github.com/CODER-7777/ICUAS26.git
cd ICUAS26
docker build -t icuas26_competition .
```

After compilation you can start a simulation:
```bash
# Execute the primary launch file / tmuxinator session
ros2 launch icuas26_competition main.launch.py
```

## 2. Algorithms and Key Modules

All planning algorithms along with other key modules are implemented in this repository:

- **Swarm Architecture**: Developed an **event-driven Finite State Machine (FSM)** for priority-based battery management and scheduling. The C++ ROS 2 node orchestrates 5 drones through `TAKEOFF` → `MISSION` → `RETURN-TO-HOME` phases, with real-time role assignment.
<p align="center">
  <img src="https://via.placeholder.com/800x225?text=FSM+Architecture" width="800" height="225"/>
</p>

- **Two-Tier Path Planning**: Designed a two-tier path planning framework integrating **BFS** with **Bresenham line-of-sight optimization** for global route generation, and **A*** for local navigation to track a moving target with uncertain position estimates. Each drone plans in priority order and inflates its smoothed path into a reservation grid, guaranteeing collision-free trajectories.
<p align="center">
  <img src="https://via.placeholder.com/400x225?text=Global+Planning" width="400" height="225"/>
  <img src="https://via.placeholder.com/400x225?text=Local+Planning" width="400" height="225"/>
</p>

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
