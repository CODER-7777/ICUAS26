#include "swarm_planner.hpp"

using namespace std::chrono_literals;

// --- OCTOMAP & GRID UTILS ---

void SwarmPlanner::fetchOctomapOnce() {
    if (!octomap_client_->wait_for_service(1s)) return;
    auto req = std::make_shared<octomap_msgs::srv::GetOctomap::Request>();
    octomap_client_->async_send_request(req,
        [this](rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedFuture future) {
            auto res = future.get();
            octomap::AbstractOcTree* abs_tree = octomap_msgs::binaryMsgToMap(res->map);
            if (abs_tree) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                tree_.reset(dynamic_cast<octomap::OcTree*>(abs_tree));
                cached_grid_ = std::make_shared<const nav_msgs::msg::OccupancyGrid>(generateOccupancyGrid());
                map_ready_ = true;
                RCLCPP_INFO(this->get_logger(), "Map Received. Grid Resolution: %.2f", cached_grid_->info.resolution);

                // Build search zones from poles detected in the octree.
                if (tree_ && !search_built_) {
                    // Record ground height (pole z_bottom = octomap minZ = drone start height)
                    double minX2, minY2, minZ2, maxX2, maxY2, maxZ2;
                    tree_->getMetricMin(minX2, minY2, minZ2);
                    tree_->getMetricMax(maxX2, maxY2, maxZ2);
                    pole_z_bottom_ = minZ2;

                    search_mgr_.buildFromOctomap(*tree_);
                    search_built_ = true;

                    // Patch any initial poses already captured: override z with ground height
                    for (auto& kv : initial_poses_) {
                        kv.second.z = pole_z_bottom_;
                    }

                    RCLCPP_INFO(this->get_logger(),
                        "SearchManager: detected %zu poles -> %zu zones. pole_z_bottom=%.3f",
                        search_mgr_.poles_.size(), search_mgr_.zones_.size(), pole_z_bottom_);
                }
            }
        });
}

nav_msgs::msg::OccupancyGrid SwarmPlanner::generateOccupancyGrid() {
    if (!tree_) return nav_msgs::msg::OccupancyGrid();

    double res = tree_->getResolution();
    double z = has_z_target_ ? z_target_from_bfs_.load() : this->get_parameter("z_target").as_double();
    double inflation_rad = this->get_parameter("inflation_radius").as_double();
    int inflation_steps = std::ceil(inflation_rad / res);

    double minX, minY, minZ, maxX, maxY, maxZ;
    tree_->getMetricMin(minX, minY, minZ);
    tree_->getMetricMax(maxX, maxY, maxZ);

    int minGx = std::floor(minX / res);
    int minGy = std::floor(minY / res);
    int w = std::ceil(maxX / res) - minGx + 1;
    int h = std::ceil(maxY / res) - minGy + 1;

    nav_msgs::msg::OccupancyGrid grid;
    grid.info.resolution = res;
    grid.info.width = w;
    grid.info.height = h;
    grid.info.origin.position.x = minGx * res;
    grid.info.origin.position.y = minGy * res;
    grid.data.assign(w * h, 0);

    octomap::point3d bbx_min((float)minX, (float)minY, (float)(z - res));
    octomap::point3d bbx_max((float)maxX, (float)maxY, (float)(z + res));

    std::vector<int> obstacles;
    for (auto it = tree_->begin_leafs_bbx(bbx_min, bbx_max); it != tree_->end_leafs_bbx(); ++it) {
        if (tree_->isNodeOccupied(*it)) {
            int lx = std::floor(it.getX() / res) - minGx;
            int ly = std::floor(it.getY() / res) - minGy;
            if (lx >= 0 && lx < w && ly >= 0 && ly < h) {
                obstacles.push_back(ly * w + lx);
            }
        }
    }

    // Simple Inflation
    for (int idx : obstacles) {
        int cy = idx / w;
        int cx = idx % w;
        for (int dy = -inflation_steps; dy <= inflation_steps; ++dy) {
            for (int dx = -inflation_steps; dx <= inflation_steps; ++dx) {
                if (dx*dx + dy*dy <= inflation_steps*inflation_steps) {
                    int nx = cx + dx;
                    int ny = cy + dy;
                    if (nx >= 0 && nx < w && ny >= 0 && ny < h) {
                        grid.data[ny * w + nx] = 100;
                    }
                }
            }
        }
    }
    return grid;
}

// --- PLANNING HELPERS ---

bool SwarmPlanner::hasGridLineOfSight(const idx& start, const idx& end, const nav_msgs::msg::OccupancyGrid& map) {
    int x0 = start.i; int y0 = start.j;
    int x1 = end.i;   int y1 = end.j;

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    int width = map.info.width;
    int height = map.info.height;

    auto isSafe = [&](int x, int y) {
        if (x < 0 || x >= width || y < 0 || y >= height) return false;
        int idx = y * width + x;
        if (map.data[idx] >= 50 || map.data[idx] == -1) return false;

        // Check neighbors for safety buffer
        int neighbors[] = {1, -1, width, -width};
        for(int n : neighbors) {
            int n_idx = idx + n;
            if (n_idx >= 0 && n_idx < width * height) {
                if (map.data[n_idx] >= 50) return false;
            }
        }
        return true;
    };

    while (true) {
        if (!isSafe(x0, y0)) return false;

        if (x0 == x1 && y0 == y1) break;

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
    return true;
}

std::vector<idx> SwarmPlanner::bfs(idx start, idx end, const nav_msgs::msg::OccupancyGrid& map) {
    int w = map.info.width;
    int h = map.info.height;
    int map_size = w * h;

    if (start.i < 0 || start.i >= w || start.j < 0 || start.j >= h ||
        end.i < 0 || end.i >= w || end.j < 0 || end.j >= h) return {};

    int start_idx = start.j * w + start.i;
    int end_idx = end.j * w + end.i;

    if (start_idx == end_idx) return {start};
    if (map.data[end_idx] >= 50 && map.data[end_idx] != -1) return {};

    // Static thread_local to avoid memory churn during frequent replanning
    static thread_local std::vector<int> g_score;
    static thread_local std::vector<int> parent;
    static thread_local std::vector<int> visited_token;
    static thread_local int current_token = 0;

    if (g_score.size() != (size_t)map_size) {
        g_score.resize(map_size);
        parent.resize(map_size);
        visited_token.resize(map_size, 0);
        current_token = 0;
    }

    current_token++;
    if (current_token == 0) {
        std::fill(visited_token.begin(), visited_token.end(), 0);
        current_token = 1;
    }

    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;

    g_score[start_idx] = 0;
    parent[start_idx] = -1;
    visited_token[start_idx] = current_token;

    int h_start = std::max(std::abs(end.i - start.i), std::abs(end.j - start.j));
    open_set.push({start_idx, h_start});

    const int dx[] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int dy[] = {0, 0, 1, -1, 1, -1, 1, -1};

    while (!open_set.empty()) {
        AStarNode current = open_set.top();
        open_set.pop();

        int curr_idx = current.idx_flat;

        if (curr_idx == end_idx) {
            std::vector<idx> path;
            int curr = end_idx;
            while (curr != -1) {
                path.push_back({curr % w, curr / w});
                curr = parent[curr];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        // Lazy skipping
        if (visited_token[curr_idx] == current_token && g_score[curr_idx] < current.f_score - std::max(std::abs(end.i - (curr_idx % w)), std::abs(end.j - (curr_idx / w)))) {
           continue;
        }

        int cx = curr_idx % w;
        int cy = curr_idx / w;

        for (int k = 0; k < 8; ++k) {
            int nx = cx + dx[k];
            int ny = cy + dy[k];

            if (nx >= 0 && nx < w && ny >= 0 && ny < h) {
                int n_idx = ny * w + nx;

                if (map.data[n_idx] >= 50 && map.data[n_idx] != -1) continue;

                int new_g = g_score[curr_idx] + 1;

                if (visited_token[n_idx] != current_token || new_g < g_score[n_idx]) {
                    g_score[n_idx] = new_g;
                    int h_cost = std::max(std::abs(end.i - nx), std::abs(end.j - ny));
                    int f_new = new_g + h_cost;

                    parent[n_idx] = curr_idx;
                    visited_token[n_idx] = current_token;
                    open_set.push({n_idx, f_new});
                }
            }
        }
    }
    return {};
}

// Inflate the line segments of a smoothed path into a planning grid as
// occupied cells. Used by prioritized A*: each drone marks its planned path
// so subsequent (lower-priority) drones treat it as a moving obstacle.
// Sub-cell line sampling guarantees no holes between widely-spaced
// line-of-sight waypoints.
void SwarmPlanner::markPathReservation(nav_msgs::msg::OccupancyGrid& grid,
                                       const std::vector<geometry_msgs::msg::Point>& path,
                                       int inflation_steps) {
    if (path.empty() || inflation_steps < 0) return;
    const double res = grid.info.resolution;
    const double ox = grid.info.origin.position.x;
    const double oy = grid.info.origin.position.y;
    const int w = grid.info.width;
    const int h = grid.info.height;
    const int r2 = inflation_steps * inflation_steps;

    auto stamp = [&](double wx, double wy) {
        int ci = static_cast<int>(std::floor((wx - ox) / res));
        int cj = static_cast<int>(std::floor((wy - oy) / res));
        for (int dy = -inflation_steps; dy <= inflation_steps; ++dy) {
            for (int dx = -inflation_steps; dx <= inflation_steps; ++dx) {
                if (dx*dx + dy*dy > r2) continue;
                int nx = ci + dx;
                int ny = cj + dy;
                if (nx >= 0 && nx < w && ny >= 0 && ny < h) {
                    grid.data[ny * w + nx] = 100;
                }
            }
        }
    };

    stamp(path[0].x, path[0].y);
    for (size_t i = 1; i < path.size(); ++i) {
        double sx = path[i-1].x, sy = path[i-1].y;
        double ex = path[i].x,   ey = path[i].y;
        double dx = ex - sx,     dy = ey - sy;
        double seg_dist = std::hypot(dx, dy);
        int steps = std::max(1, static_cast<int>(std::ceil(seg_dist / (res * 0.5))));
        for (int s = 1; s <= steps; ++s) {
            double t = static_cast<double>(s) / steps;
            stamp(sx + dx * t, sy + dy * t);
        }
    }
}

std::vector<geometry_msgs::msg::Point> SwarmPlanner::runPlanningPipeline(
    const nav_msgs::msg::OccupancyGrid& map,
    geometry_msgs::msg::Point target,
    geometry_msgs::msg::Point current_pos,
    double target_z)
{
    const double res = map.info.resolution;
    const double ox = map.info.origin.position.x;
    const double oy = map.info.origin.position.y;
    // target_z is now an argument


    auto gToW = [&](int i, int j) {
        geometry_msgs::msg::Point p;
        p.x = ox + (i + 0.5) * res;
        p.y = oy + (j + 0.5) * res;
        p.z = target_z;
        return p;
    };

    idx start_idx = {(int)std::floor((current_pos.x - ox) / res), (int)std::floor((current_pos.y - oy) / res)};
    idx end_idx = {(int)std::floor((target.x - ox) / res), (int)std::floor((target.y - oy) / res)};

    auto grid_path = bfs(start_idx, end_idx, map);
    if (grid_path.empty()) return {};

    std::vector<geometry_msgs::msg::Point> drone_pts;
    drone_pts.reserve(grid_path.size() / get_num_robots());

    drone_pts.push_back(gToW(grid_path[0].i, grid_path[0].j));

    size_t path_idx = 0;
    while (path_idx < grid_path.size() - 1) {
        bool found_jump = false;

        for (size_t j = grid_path.size() - 1; j > path_idx; --j) {
            // Now hasGridLineOfSight is defined before this call
            if (hasGridLineOfSight(grid_path[path_idx], grid_path[j], map)) {
                drone_pts.push_back(gToW(grid_path[j].i, grid_path[j].j));
                path_idx = j;
                found_jump = true;
                break;
            }
        }

        if (!found_jump) {
            path_idx++;
            drone_pts.push_back(gToW(grid_path[path_idx].i, grid_path[path_idx].j));
        }
    }
    return drone_pts;
}
