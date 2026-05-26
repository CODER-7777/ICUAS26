#pragma once

#include "types.hpp"

#include <octomap/octomap.h>
#include <geometry_msgs/msg/point.hpp>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <functional>
#include <unordered_set>

class SearchManager {
public:
    // Tunables (could be exposed as ROS params later).
    double standoff_ = 1.5;       // distance from pole surface to zone centre
    double visit_radius_ = 0.5;   // drone must be this close to mark visited
    double zone_height_low_ = 0.25;  // 25% of pole height
    double zone_height_high_ = 0.75; // 75% of pole height
    double min_pole_cells_ = 4;   // floor area threshold to count as pole
    double max_pole_extent_ = 1.5; // m; reject components wider than this (walls)
    double dwell_time_sec_ = 0.5; // seconds to dwell at a zone before marking visited

    // Cooperative planning tunables.
    double drone_clearance_ = 0.6;  // min XY distance between zone and any drone obstacle
    double pillar_clearance_ = 0.3; // extra radial buffer around pole when pillar-checking

    std::vector<PoleInfo> poles_;
    std::vector<ViewZone> zones_;

    // Locked zones: indices currently claimed by a drone this planning tick.
    // resetLocks() at start of tick; lockZone(i) when a drone is assigned.
    std::unordered_set<int> locked_zones_;

    // Octree pointer for 3D occupancy queries (pillar skip).
    // Lifetime managed by caller; we hold a non-owning view.
    const octomap::OcTree* tree_ = nullptr;

    void buildFromOctomap(const octomap::OcTree& tree);
    void markVisited(double x, double y, double z);

    // Prioritized cooperative zone pick.
    //   drone_obstacles: positions of all OTHER drones (chain + idle already
    //     planned this tick). Zone rejected if within drone_clearance_ XY.
    //   Zone also rejected if locked, on a pillar, or its target cell is
    //     occupied in the octomap at that z.
    int pickNextZone(
        double drone_x, double drone_y, double drone_z,
        const std::vector<geometry_msgs::msg::Point>& anchors,
        double comm_range,
        const std::function<bool(double, double, double, double)>& losCheck,
        const std::vector<geometry_msgs::msg::Point>& drone_obstacles) const;

    // Backwards-compatible wrapper (no drone obstacles).
    int pickNextZone(
        double drone_x, double drone_y, double drone_z,
        const std::vector<geometry_msgs::msg::Point>& anchors,
        double comm_range,
        const std::function<bool(double, double, double, double)>& losCheck) const;

    // Lock / unlock zones across a planning tick.
    void resetLocks() { locked_zones_.clear(); }
    void lockZone(int idx) { locked_zones_.insert(idx); }
    bool isLocked(int idx) const { return locked_zones_.count(idx) != 0; }

    // True if (x,y,z) lies on / inside a pillar (any pole's XY footprint
    // expanded by pillar_clearance_, within the pole's vertical extent).
    bool isOnPillar(double x, double y, double z) const;

    // True if octomap reports the cell at (x,y,z) as occupied. False if
    // tree_ unset, out of bounds, or cell unknown.
    bool isOctomapOccupied(double x, double y, double z) const;

    bool allVisited() const;
};
