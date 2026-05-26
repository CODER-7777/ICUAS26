#pragma once

#include "types.hpp"

#include <octomap/octomap.h>
#include <geometry_msgs/msg/point.hpp>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <functional>

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

    std::vector<PoleInfo> poles_;
    std::vector<ViewZone> zones_;

    void buildFromOctomap(const octomap::OcTree& tree);
    void markVisited(double x, double y, double z);
    int pickNextZone(
        double drone_x, double drone_y, double drone_z,
        const std::vector<geometry_msgs::msg::Point>& anchors,
        double comm_range,
        const std::function<bool(double, double, double, double)>& losCheck) const;
    bool allVisited() const;
};
