# Autonomous Multi-UAV Swarm System for Urban Search & Rescue

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros)](https://docs.ros.org/en/humble/)
[![C++](https://img.shields.io/badge/C++-17-00599C?logo=c%2B%2B)](https://cplusplus.com/)
[![Python](https://img.shields.io/badge/Python-3.10-3776AB?logo=python)](https://www.python.org/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Garden-FF7B00?logo=gazebo)](https://gazebosim.org/)
[![Docker](https://img.shields.io/badge/Docker-Ready-2496ED?logo=docker)](https://www.docker.com/)

> **Developed for the ICUAS 2026 International Competition**

This repository contains the complete software stack for an autonomous multi-drone swarm system. A fleet of 5 Crazyflie nano-drones cooperatively navigates an urban cityscape to locate ArUco markers on obstacle pillars while maintaining a communication relay chain to a moving ground vehicle (AGV). 

The system operates entirely in **ROS 2 (Humble)** with **Gazebo Garden** simulation, features a custom C++ swarm planner, and is fully Docker-containerized for reproducibility.

![Swarm Simulation Demo](https://via.placeholder.com/800x400?text=Insert+GIF/Screenshot+of+Swarm+Here)

---

## Competition Results

- **Global Rank 6:** Emerged as the **only Indian team** to qualify for the final round in Greece out of a highly competitive international pool.
- **Mission Success:** Achieved detection of **7/8 ArUco markers** in a complex 100m × 100m × 100m 3D map using a 5-drone swarm system.
- **Performance Metrics:** Sustained over **70% communication connectivity** with a moving target with uncertain position estimates, and improved flight time by **40%** through optimized battery scheduling.

---

## Key Features

### Event-Driven Swarm Architecture
Developed an **event-driven Finite State Machine (FSM)** for priority-based battery management and scheduling. The C++ ROS 2 node orchestrates 5 drones through `TAKEOFF` → `MISSION` → `RETURN-TO-HOME` phases, with real-time role assignment (Chain Component, Search, RTH, Landing).

### Two-Tier Path Planning Framework
Designed a two-tier path planning framework integrating **BFS** with **Bresenham line-of-sight optimization** for global route generation, and **A*** for local navigation to track a moving target with uncertain position estimates. Each drone plans in priority order and inflates its smoothed path into a reservation grid, guaranteeing collision-free trajectories.

### Communication-Constrained Relay Chain
Solves a connectivity-constrained multi-robot problem where drones must maintain a line-of-sight (LOS) relay chain between a base station and a moving AGV. Utilized **bipartite matching** for drone-to-waypoint assignment, achieving an optimized **O(log D · VE)** execution time to minimize the bottleneck edge length across the fleet.

### Autonomous ArUco Search
A `SearchManager` module extracts vertical obstacle pillars from OctoMap data, generates 8 viewpoint zones per pillar (4 cardinal directions × 2 heights), and assigns idle drones to unvisited zones using cooperative selection with drone-clearance and pillar-clearance constraints.

### Battery Management System (BMS)
Features a hot-swap charging strategy with concurrent slot management (max 2 drones charging simultaneously). Includes smart recall logic to only recall a charging drone if it has gained ≥10% charge or all other drones are unavailable, plus an emergency RTH triggered below 25% battery.

### Advanced Control Prototypes
- **RL-Based Pursuit:** A PPO-based reinforcement learning node using Stable Baselines3 for single-drone obstacle-avoidant tracking, utilizing 12-ray LIDAR-like raycasting.
- **Distributed MPC:** A Model Predictive Control node using CVXOPT for QP-based obstacle-avoidant trajectory optimization with RRT* initial positioning.

---

## Technology Stack

- **Frameworks:** ROS 2 Humble, Gazebo Garden
- **Languages:** C++17, Python 3
- **Algorithms:** A*, BFS, RRT*, Hungarian Matching, Raycasting
- **Libraries:** OctoMap, Stable Baselines3, CVXOPT
- **Infrastructure:** Docker, tmuxinator, CrazySim

---

## Getting Started

### Prerequisites
- Docker Engine & Docker Compose
- NVIDIA GPU with Container Toolkit (recommended for Gazebo)

### Installation & Execution
1. **Clone the repository:**
   ```bash
   git clone https://github.com/yourusername/autonomous-drone-swarm.git
   cd autonomous-drone-swarm
   ```
2. **Build the Docker container:**
   ```bash
   docker build -t icuas26_competition .
   ```
3. **Run the simulation stack:**
   *(Instructions on how to launch the primary launch file / tmuxinator session)*
   ```bash
   # Example
   ros2 launch icuas26_competition main.launch.py
   ```

---

## Repository Structure

```text
icuas26_competition/
├── Dockerfile                  # Containerization setup
├── config.yaml                 # Core configuration parameters
├── workspace/                  
│   └── src/                    # ROS 2 packages
│       ├── aruco_mission_cpp/  # ArUco detection & pose estimation
│       ├── graph_traversal/    # Swarm planner, A*, BMS, & Search logic
│       ├── debug_tools/        # Telemetry & debugging nodes
│       └── swarm_viz/          # Foxglove & RViz visualizers
├── scripts/                    
│   ├── AGV.py                  # Ground vehicle waypoint interpolation
│   └── charging.py             # Gazebo battery state & docking logic
├── worlds/                     # Gazebo Garden city environments
└── ...
```

---

## Author

**Mansoju Vivekananda**  
*(Add links to your LinkedIn, Portfolio, or email here)*
