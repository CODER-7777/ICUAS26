#include "search_manager.hpp"

void SearchManager::buildFromOctomap(const octomap::OcTree& tree) {
    poles_.clear();
    zones_.clear();
    locked_zones_.clear();
    tree_ = &tree;

    double res = tree.getResolution();
    double minX, minY, minZ, maxX, maxY, maxZ;
    const_cast<octomap::OcTree&>(tree).getMetricMin(minX, minY, minZ);
    const_cast<octomap::OcTree&>(tree).getMetricMax(maxX, maxY, maxZ);

    // Floor slab: 0.3 m thick starting ~0.2 m above the ground. Poles
    // are guaranteed to be present here, while ground noise above the
    // physical floor is uncommon.
    double z_slab_lo = minZ + 0.2;
    double z_slab_hi = std::min(z_slab_lo + 0.3, maxZ);

    int minGx = std::floor(minX / res);
    int minGy = std::floor(minY / res);
    int w = std::ceil(maxX / res) - minGx + 1;
    int h = std::ceil(maxY / res) - minGy + 1;

    std::vector<uint8_t> occ(w * h, 0);
    octomap::point3d bbx_min((float)minX, (float)minY, (float)z_slab_lo);
    octomap::point3d bbx_max((float)maxX, (float)maxY, (float)z_slab_hi);
    for (auto it = const_cast<octomap::OcTree&>(tree).begin_leafs_bbx(bbx_min, bbx_max);
         it != const_cast<octomap::OcTree&>(tree).end_leafs_bbx(); ++it) {
        if (!const_cast<octomap::OcTree&>(tree).isNodeOccupied(*it)) continue;
        int lx = std::floor(it.getX() / res) - minGx;
        int ly = std::floor(it.getY() / res) - minGy;
        if (lx >= 0 && lx < w && ly >= 0 && ly < h) {
            occ[ly * w + lx] = 1;
        }
    }

    // Flood-fill connected components (4-connected; pole cross-sections
    // are small enough that 4-conn vs 8-conn makes no real difference).
    std::vector<int> label(w * h, -1);
    int next_label = 0;
    for (int j = 0; j < h; ++j) {
        for (int i = 0; i < w; ++i) {
            int idx = j * w + i;
            if (!occ[idx] || label[idx] != -1) continue;
            // BFS
            std::queue<int> q;
            q.push(idx);
            label[idx] = next_label;
            std::vector<int> comp_cells;
            int min_i = i, max_i = i, min_j = j, max_j = j;
            while (!q.empty()) {
                int cur = q.front(); q.pop();
                comp_cells.push_back(cur);
                int ci = cur % w, cj = cur / w;
                min_i = std::min(min_i, ci); max_i = std::max(max_i, ci);
                min_j = std::min(min_j, cj); max_j = std::max(max_j, cj);
                const int di[4] = {1, -1, 0, 0};
                const int dj[4] = {0, 0, 1, -1};
                for (int k = 0; k < 4; ++k) {
                    int ni = ci + di[k], nj = cj + dj[k];
                    if (ni < 0 || ni >= w || nj < 0 || nj >= h) continue;
                    int nidx = nj * w + ni;
                    if (!occ[nidx] || label[nidx] != -1) continue;
                    label[nidx] = next_label;
                    q.push(nidx);
                }
            }
            next_label++;

            // Reject too-small (noise) or too-large (walls / fences).
            if ((int)comp_cells.size() < min_pole_cells_) continue;
            double extent_x = (max_i - min_i + 1) * res;
            double extent_y = (max_j - min_j + 1) * res;
            if (extent_x > max_pole_extent_ || extent_y > max_pole_extent_) continue;

            // Centroid in world coords.
            double sx = 0.0, sy = 0.0;
            for (int c : comp_cells) {
                int ci = c % w, cj = c / w;
                sx += (minGx + ci + 0.5) * res;
                sy += (minGy + cj + 0.5) * res;
            }
            sx /= comp_cells.size();
            sy /= comp_cells.size();

            PoleInfo p;
            p.cx = sx;
            p.cy = sy;
            p.radius = 0.5 * std::max(extent_x, extent_y);
            p.z_bottom = minZ;
            p.z_top = maxZ; // assume pole spans full vertical extent
            poles_.push_back(p);
        }
    }

    // Generate 8 zones per pole (4 cardinal x 2 heights).
    const double dirs[4][2] = { {1,0}, {0,1}, {-1,0}, {0,-1} }; // E, N, W, S
    for (size_t pi = 0; pi < poles_.size(); ++pi) {
        const auto& p = poles_[pi];
        double height = std::max(0.5, p.z_top - p.z_bottom);
        double z_lo = p.z_bottom + zone_height_low_ * height;
        double z_hi = p.z_bottom + zone_height_high_ * height;
        for (int d = 0; d < 4; ++d) {
            double r = p.radius + standoff_;
            for (int hidx = 0; hidx < 2; ++hidx) {
                ViewZone z;
                z.x = p.cx + r * dirs[d][0];
                z.y = p.cy + r * dirs[d][1];
                z.z = (hidx == 0) ? z_lo : z_hi;
                z.yaw = std::atan2(p.cy - z.y, p.cx - z.x); // face the pole
                z.pole_idx = (int)pi;
                z.dir_idx = d;
                z.height_idx = hidx;
                zones_.push_back(z);
            }
        }
    }
}

void SearchManager::markVisited(double x, double y, double z) {
    auto now = std::chrono::steady_clock::now();
    for (auto& zn : zones_) {
        if (zn.visited) continue;
        double dx = zn.x - x;
        double dy = zn.y - y;
        double dz = zn.z - z;
        if (std::sqrt(dx*dx + dy*dy + dz*dz) > visit_radius_) continue;
        // Side check: drone must be on the same side of the pole as
        // the zone (dot of zone->pole offset with drone->pole offset).
        const auto& p = poles_[zn.pole_idx];
        double zone_off_x = zn.x - p.cx;
        double zone_off_y = zn.y - p.cy;
        double drone_off_x = x - p.cx;
        double drone_off_y = y - p.cy;
        if (zone_off_x * drone_off_x + zone_off_y * drone_off_y <= 0.0) continue;
        // Dwell: on first arrival start the timer; mark visited only after
        // dwell_time_sec_ has elapsed, giving the drone time to observe.
        if (zn.dwell_start_ == std::chrono::steady_clock::time_point{}) {
            zn.dwell_start_ = now;
        }
        auto elapsed = std::chrono::duration<double>(now - zn.dwell_start_).count();
        if (elapsed >= dwell_time_sec_) {
            zn.visited = true;
        }
    }
}

bool SearchManager::isOnPillar(double x, double y, double z) const {
    for (const auto& p : poles_) {
        if (z < p.z_bottom || z > p.z_top) continue;
        double dx = x - p.cx;
        double dy = y - p.cy;
        double r = p.radius + pillar_clearance_;
        if (dx*dx + dy*dy <= r*r) return true;
    }
    return false;
}

bool SearchManager::isOctomapOccupied(double x, double y, double z) const {
    if (!tree_) return false;
    octomap::OcTreeNode* node = const_cast<octomap::OcTree*>(tree_)
        ->search(octomap::point3d((float)x, (float)y, (float)z));
    if (!node) return false;
    return const_cast<octomap::OcTree*>(tree_)->isNodeOccupied(node);
}

int SearchManager::pickNextZone(
    double drone_x, double drone_y, double drone_z,
    const std::vector<geometry_msgs::msg::Point>& anchors,
    double comm_range,
    const std::function<bool(double, double, double, double)>& losCheck,
    const std::vector<geometry_msgs::msg::Point>& drone_obstacles) const
{
    int best = -1;
    double best_d = 1e18;
    const double dc2 = drone_clearance_ * drone_clearance_;

    for (size_t i = 0; i < zones_.size(); ++i) {
        const auto& zn = zones_[i];
        if (zn.visited) continue;

        // 1. Locked by another drone this tick.
        if (isLocked((int)i)) continue;

        // 2. Skip if zone position sits on a pillar.
        if (isOnPillar(zn.x, zn.y, zn.z)) continue;

        // 3. Skip if octomap reports the cell occupied (full 3D check).
        if (isOctomapOccupied(zn.x, zn.y, zn.z)) continue;

        // 4. Skip if any other drone (chain or already-planned idle) is
        //    inside drone_clearance_ XY of this zone at a similar z.
        bool blocked_by_drone = false;
        for (const auto& o : drone_obstacles) {
            double ox = zn.x - o.x;
            double oy = zn.y - o.y;
            double oz = zn.z - o.z;
            // Same altitude band: hard XY clearance.
            if (std::abs(oz) < 0.6 && (ox*ox + oy*oy) < dc2) {
                blocked_by_drone = true;
                break;
            }
        }
        if (blocked_by_drone) continue;

        // 5. LOS / range constraint: must be visible from at least one anchor.
        bool anchored = false;
        for (const auto& a : anchors) {
            double dx = zn.x - a.x, dy = zn.y - a.y, dz = zn.z - a.z;
            if (std::sqrt(dx*dx + dy*dy + dz*dz) > comm_range) continue;
            if (losCheck(a.x, a.y, zn.x, zn.y)) { anchored = true; break; }
        }
        if (!anchored) continue;

        // 6. Cost = 3D distance from drone.
        double dx = zn.x - drone_x, dy = zn.y - drone_y, dz = zn.z - drone_z;
        double d = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (d < best_d) { best_d = d; best = (int)i; }
    }
    return best;
}

int SearchManager::pickNextZone(
    double drone_x, double drone_y, double drone_z,
    const std::vector<geometry_msgs::msg::Point>& anchors,
    double comm_range,
    const std::function<bool(double, double, double, double)>& losCheck) const
{
    static const std::vector<geometry_msgs::msg::Point> empty;
    return pickNextZone(drone_x, drone_y, drone_z, anchors, comm_range, losCheck, empty);
}

bool SearchManager::allVisited() const {
    for (const auto& z : zones_) if (!z.visited) return false;
    return !zones_.empty();
}
