
---

## 1️Starting ArUco Detection

Launch the ArUco detection node to detect markers and publish their poses.

```bash
ros2 run aruco_mission aruco_mission_node
```

**Purpose**

* Detects ArUco markers from the camera stream
* Publishes marker IDs and poses for downstream tasks

---

## 2️ Waypoint Follower

Run the waypoint follower node to navigate through generated waypoints.

```bash
ros2 run mission_follower waypoint_follower
```

**Purpose**

* Subscribes to waypoint data
* Commands the robot/drone to follow the waypoint sequence

---

## 3️ Graph Generation from OctoMap

Generate a navigation graph / waypoints from the OctoMap representation.

```bash
ros2 run octomap_map_generator waypoint_extractor_node
```

**Purpose**

* Processes OctoMap data
* Extracts navigable waypoints or graph structure for planning

---
