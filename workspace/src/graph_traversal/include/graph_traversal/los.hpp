#ifndef GRAPH_TRAVERSAL__LOS_HPP_
#define GRAPH_TRAVERSAL__LOS_HPP_

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <cmath>

namespace graph_traversal {

// 2D Bresenham line-of-sight check on an OccupancyGrid in world coordinates.
// A cell blocks LOS if it (or any 4-neighbor, as a safety buffer) is occupied
// (>= 50) or unknown (== -1). Mirrors SwarmPlanner::hasGridLineOfSight so the
// planner and swarm_viz see identical comm topology.
inline bool hasGridLineOfSightWorld(double ax, double ay,
                                    double bx, double by,
                                    const nav_msgs::msg::OccupancyGrid & grid) {
    const double res = grid.info.resolution;
    if (res <= 0.0) return false;
    const double ox = grid.info.origin.position.x;
    const double oy = grid.info.origin.position.y;
    const int width  = static_cast<int>(grid.info.width);
    const int height = static_cast<int>(grid.info.height);

    int x0 = static_cast<int>(std::floor((ax - ox) / res));
    int y0 = static_cast<int>(std::floor((ay - oy) / res));
    int x1 = static_cast<int>(std::floor((bx - ox) / res));
    int y1 = static_cast<int>(std::floor((by - oy) / res));

    auto isSafe = [&](int x, int y) {
        if (x < 0 || x >= width || y < 0 || y >= height) return false;
        const int idx = y * width + x;
        const auto v = grid.data[idx];
        if (v >= 50 || v == -1) return false;
        const int neighbors[4] = {1, -1, width, -width};
        for (int n : neighbors) {
            const int n_idx = idx + n;
            if (n_idx >= 0 && n_idx < width * height) {
                if (grid.data[n_idx] >= 50) return false;
            }
        }
        return true;
    };

    const int dx = std::abs(x1 - x0);
    const int dy = std::abs(y1 - y0);
    const int sx = (x0 < x1) ? 1 : -1;
    const int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        if (!isSafe(x0, y0)) return false;
        if (x0 == x1 && y0 == y1) break;
        const int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
    return true;
}

}  // namespace graph_traversal

#endif  // GRAPH_TRAVERSAL__LOS_HPP_
