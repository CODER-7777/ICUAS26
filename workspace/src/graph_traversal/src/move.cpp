#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <octomap/octomap.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <crazyflie_interfaces/msg/position.hpp>
#include <crazyflie_interfaces/srv/takeoff.hpp>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <mutex>
#include <queue>
#include <map>
#include <unordered_set>
#include <omp.h>

using namespace std::chrono_literals;

struct idx { int i, j; };

// Node for A* Priority Queue
struct AStarNode {
    int idx_flat;
    int f_score; 
    bool operator>(const AStarNode& other) const { return f_score > other.f_score; }
};

enum class SwarmState { TAKEOFF, MISSION };

class SwarmPlanner : public rclcpp::Node {
public:
    SwarmPlanner() : Node("Swarm_planner") {
        this->declare_parameter<double>("inflation_radius", 0.3); // Reduced slightly for better fit
        this->declare_parameter<double>("z_target", 1.0);
        current_state_ = SwarmState::TAKEOFF;

        // Create a Reentrant Callback Group to allow parallelism
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_;

        // 1. Target Subscriber - ALWAYS updates the latest points
        dronePos_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/drone/waypoint_array", 1, 
            [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                current_bfsPoints_ = *msg;
                hasPoints_ = true;
                // Optional: Log when points change to verify updates
                // RCLCPP_INFO(this->get_logger(), "Received %zu new targets", msg->poses.size());
            }, sub_opt);

        // 2. Drone State Subscriptions
        for (const std::string& id : droneIds_) {
            drone_subs_.push_back(this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/" + id + "/pose", 10,
                [this, id](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    this->swarm_poses_[id] = *msg;
                }, sub_opt));

            cmd_pubs_[id] = this->create_publisher<crazyflie_interfaces::msg::Position>(
                "/" + id + "/cmd_position", 10);
        }

        // 3. Clients
        octomap_client_ = this->create_client<octomap_msgs::srv::GetOctomap>("octomap_binary", 
            rmw_qos_profile_services_default, callback_group_);
        
        takeoff_client_ = this->create_client<crazyflie_interfaces::srv::Takeoff>("/all/takeoff",
            rmw_qos_profile_services_default, callback_group_);

        // 4. Timers
        // Init: Get map once
        init_timer_ = this->create_wall_timer(500ms, [this]() { if (!map_ready_) fetchOctomapOnce(); });
        
        // Planning Loop (High Level Logic)
        timer_ = this->create_wall_timer(100ms, std::bind(&SwarmPlanner::runSwarmSystem, this), callback_group_);
        
        // Control Loop (Publishing commands fast)
        control_timer_ = this->create_wall_timer(50ms, std::bind(&SwarmPlanner::publishCommands, this), callback_group_);
    }

private:
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::TimerBase::SharedPtr timer_, init_timer_, control_timer_;
    rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedPtr octomap_client_;
    rclcpp::Client<crazyflie_interfaces::srv::Takeoff>::SharedPtr takeoff_client_;
    
    SwarmState current_state_;
    bool takeoff_called_ = false;
    const double REACH_THRESHOLD = 0.15;
    
    std::mutex data_mutex_;
    std::unique_ptr<octomap::OcTree> tree_;
    nav_msgs::msg::OccupancyGrid cached_grid_;
    geometry_msgs::msg::PoseArray current_bfsPoints_;
    bool hasPoints_ = false;
    bool map_ready_ = false;

    std::vector<std::string> droneIds_ = {"cf_1", "cf_2", "cf_3", "cf_4", "cf_5"};
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> drone_subs_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr dronePos_sub_;
    std::map<std::string, geometry_msgs::msg::PoseStamped> swarm_poses_;
    std::map<std::string, rclcpp::Publisher<crazyflie_interfaces::msg::Position>::SharedPtr> cmd_pubs_;
    std::map<std::string, std::vector<crazyflie_interfaces::msg::Position>> active_commands_;

    // --- OCTOMAP & GRID UTILS ---

    void fetchOctomapOnce() {
        if (!octomap_client_->wait_for_service(1s)) return;
        auto req = std::make_shared<octomap_msgs::srv::GetOctomap::Request>();
        octomap_client_->async_send_request(req, 
            [this](rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedFuture future) {
                auto res = future.get();
                octomap::AbstractOcTree* abs_tree = octomap_msgs::binaryMsgToMap(res->map);
                if (abs_tree) {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    tree_.reset(dynamic_cast<octomap::OcTree*>(abs_tree));
                    cached_grid_ = generateOccupancyGrid();
                    map_ready_ = true;
                    RCLCPP_INFO(this->get_logger(), "Map Received. Grid Resolution: %.2f", cached_grid_.info.resolution);
                }
            });
    }

    nav_msgs::msg::OccupancyGrid generateOccupancyGrid() {
        if (!tree_) return nav_msgs::msg::OccupancyGrid();
        
        double res = tree_->getResolution();
        double z = this->get_parameter("z_target").as_double();
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

    // --- PLANNING HELPERS (Order Matters!) ---

    bool hasGridLineOfSight(const idx& start, const idx& end, const nav_msgs::msg::OccupancyGrid& map) {
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

    std::vector<idx> bfs(idx start, idx end, const nav_msgs::msg::OccupancyGrid& map) {
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

    std::vector<geometry_msgs::msg::Point> runPlanningPipeline(const nav_msgs::msg::OccupancyGrid& map, geometry_msgs::msg::Point target, geometry_msgs::msg::Point current_pos) {
        const double res = map.info.resolution;
        const double ox = map.info.origin.position.x;
        const double oy = map.info.origin.position.y;
        const double target_z = this->get_parameter("z_target").as_double();

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
        drone_pts.reserve(grid_path.size() / 5); 

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

    // --- MAIN LOGIC ---

    bool canMatch(int u, double threshold, const std::vector<std::vector<double>>& costs, 
              std::vector<int>& match, std::vector<bool>& vis) {
        for (size_t v = 0; v < costs[0].size(); ++v) {
            if (costs[u][v] <= threshold && !vis[v]) {
                vis[v] = true;
                if (match[v] < 0 || canMatch(match[v], threshold, costs, match, vis)) {
                    match[v] = u;
                    return true;
                }
            }
        }
        return false;
    }

    void runSwarmSystem() {
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (swarm_poses_.size() < droneIds_.size() || !map_ready_) return;
        }

        if (current_state_ == SwarmState::TAKEOFF) {
            handleTakeoff();
        } else if (current_state_ == SwarmState::MISSION) {
            handleMission();
        }
    }

    void handleTakeoff() {
        if (!takeoff_called_) {
            if (!takeoff_client_->wait_for_service(1s)) return;
            auto req = std::make_shared<crazyflie_interfaces::srv::Takeoff::Request>();
            req->height = 1.0; 
            req->duration.sec = 2;
            req->duration.nanosec = 0;
            takeoff_client_->async_send_request(req);
            takeoff_called_ = true;
            return;
        }

        bool all_up = true;
        double z_target = this->get_parameter("z_target").as_double();
        
        std::lock_guard<std::mutex> lock(data_mutex_); // Protect swarm_poses_
        for (const auto& id : droneIds_) {
            if (std::abs(swarm_poses_[id].pose.position.z - z_target) > REACH_THRESHOLD) all_up = false;
        }

        if (all_up && hasPoints_) {
            RCLCPP_INFO(this->get_logger(), "Transitioning to MISSION");
            current_state_ = SwarmState::MISSION;
        }
    }

    void handleMission() {
        nav_msgs::msg::OccupancyGrid local_grid;
        geometry_msgs::msg::PoseArray local_targets;
        std::map<std::string, geometry_msgs::msg::PoseStamped> local_poses;

        // CRITICAL: This snapshot ensures we always work on the LATEST points
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            local_grid = cached_grid_;
            local_targets = current_bfsPoints_; // If new points came in, we get them here
            local_poses = swarm_poses_;
        }

        const size_t num_drones = droneIds_.size();
        const size_t num_targets = local_targets.poses.size();
        
        // --- 1. Matching Logic (Cost Matrix & Binary Search) ---
        std::vector<std::vector<double>> costs(num_drones, std::vector<double>(num_targets));
        std::vector<double> all_distances;

        for (size_t i = 0; i < num_drones; ++i) {
            for (size_t j = 0; j < num_targets; ++j) {
                auto& d_pos = local_poses[droneIds_[i]].pose.position;
                auto& t_pos = local_targets.poses[j].position;
                double dist = std::hypot(d_pos.x - t_pos.x, d_pos.y - t_pos.y);
                costs[i][j] = dist;
                all_distances.push_back(dist);
            }
        }
        std::sort(all_distances.begin(), all_distances.end());

        int low = 0, high = all_distances.size() - 1;
        std::vector<int> final_match(num_targets, -1);
        double best_threshold = (all_distances.empty()) ? 0.0 : all_distances.back();
        int required_matches = std::min(num_drones, num_targets);

        while (low <= high) {
            int mid = low + (high - low) / 2;
            double threshold = all_distances[mid];
            std::vector<int> current_match(num_targets, -1);
            int matches_found = 0;
            
            for (size_t i = 0; i < num_drones; ++i) {
                std::vector<bool> vis(num_targets, false);
                if (canMatch(i, threshold, costs, current_match, vis)) matches_found++;
            }

            if (matches_found >= required_matches) {
                best_threshold = threshold;
                final_match = current_match; 
                high = mid - 1;
            } else {
                low = mid + 1;
            }
        }

        std::map<int, int> drone_to_target; 
        for (size_t t_idx = 0; t_idx < final_match.size(); ++t_idx) {
            if (final_match[t_idx] != -1) drone_to_target[final_match[t_idx]] = t_idx;
        }

        // --- 2. Parallel Path Planning ---
        struct DroneJob {
            std::string id;
            bool is_assigned;
            geometry_msgs::msg::Point target_pos; 
            double target_z;
        };

        std::vector<DroneJob> jobs(num_drones);
        int idle_counter = 0;
        const double z_mission = this->get_parameter("z_target").as_double();
        const double stack_base_alt = 2.0;
        const double stack_interval = 0.5;

        // 4a. Build list of "Connection Anchors" (Base + Active Targets)
        std::vector<geometry_msgs::msg::Point> anchors;
        
        // Anchor 0: Base Station (Always connected)
        geometry_msgs::msg::Point base;
        base.x = 0.0; base.y = 0.0; base.z = z_mission;
        anchors.push_back(base);

        // Add all active mission targets as anchors
        for(const auto& pose : local_targets.poses) {
            anchors.push_back(pose.position);
        }

        for (size_t i = 0; i < num_drones; ++i) {
            jobs[i].id = droneIds_[i];
            auto current_pos = local_poses[droneIds_[i]].pose.position;

            if (drone_to_target.count(i)) {
                // --- ACTIVE DRONE ---
                jobs[i].is_assigned = true;
                int t_idx = drone_to_target[i];
                jobs[i].target_pos = local_targets.poses[t_idx].position;
                jobs[i].target_z = z_mission;
            } else {
                // --- IDLE DRONE (SHADOW STRATEGY) ---
                jobs[i].is_assigned = false;
                
                // 1. Find the Nearest Anchor (Base or Active Drone)
                int best_anchor_idx = 0;
                double min_dist = 1e9;

                for (size_t k = 0; k < anchors.size(); ++k) {
                    double d = std::hypot(current_pos.x - anchors[k].x, current_pos.y - anchors[k].y);
                    if (d < min_dist) {
                        min_dist = d;
                        best_anchor_idx = k;
                    }
                }

                // 2. Assign Target to that Anchor
                geometry_msgs::msg::Point shadow_target = anchors[best_anchor_idx];

                // 3. Collision Avoidance Offset
                double offset_dist = 2.0;
                
                if (i % 4 == 0)      shadow_target.x += offset_dist;
                else if (i % 4 == 1) shadow_target.x -= offset_dist;
                else if (i % 4 == 2) shadow_target.y += offset_dist;
                else                 shadow_target.y -= offset_dist;

                jobs[i].target_pos = shadow_target;
                jobs[i].target_z = z_mission; 
            }
        }

        std::vector<std::pair<std::string, std::vector<crazyflie_interfaces::msg::Position>>> new_commands(num_drones);

        #pragma omp parallel for
        for (size_t i = 0; i < num_drones; ++i) {
            const auto& job = jobs[i];
            auto current_pos = local_poses[job.id].pose.position;
            std::vector<crazyflie_interfaces::msg::Position> drone_path_cmds;

            double dist_to_target = std::hypot(current_pos.x - job.target_pos.x, current_pos.y - job.target_pos.y);
            
            // "Lazy" check: If unassigned and already at the shadow spot, just hold.
            if (!job.is_assigned && dist_to_target < 0.2) {
                 crazyflie_interfaces::msg::Position cmd;
                 cmd.x = current_pos.x; cmd.y = current_pos.y; cmd.z = job.target_z; cmd.yaw = 0.0;
                 drone_path_cmds.push_back(cmd);
            } else {
                 // Plan path to the shadow spot to avoid obstacles
                 auto path = runPlanningPipeline(local_grid, job.target_pos, current_pos);
                 
                 if (!path.empty()) {
                    for (size_t k = 0; k < path.size(); ++k) {
                        crazyflie_interfaces::msg::Position cmd;
                        cmd.x = path[k].x; cmd.y = path[k].y; cmd.z = job.target_z;
                        
                        double dx, dy;
                        if (k == 0) { dx = path[k].x - current_pos.x; dy = path[k].y - current_pos.y; }
                        else { dx = path[k].x - path[k-1].x; dy = path[k].y - path[k-1].y; }
                        
                        if (std::hypot(dx, dy) > 0.01) cmd.yaw = std::atan2(dy, dx);
                        else cmd.yaw = (drone_path_cmds.empty()) ? 0.0 : drone_path_cmds.back().yaw;
                        
                        drone_path_cmds.push_back(cmd);
                    }
                 } else {
                    crazyflie_interfaces::msg::Position cmd;
                    cmd.x = current_pos.x; cmd.y = current_pos.y; cmd.z = job.target_z; cmd.yaw = 0.0;
                    drone_path_cmds.push_back(cmd);
                 }
            }
            new_commands[i] = {job.id, drone_path_cmds};
        }

        // --- 3. Atomic Commit ---
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            active_commands_.clear(); 
            for (const auto& cmd_pair : new_commands) {
                active_commands_[cmd_pair.first] = cmd_pair.second;
            }
        }
    }

    geometry_msgs::msg::Point applyRepulsion(
        geometry_msgs::msg::Point target,
        geometry_msgs::msg::Point current,
        const std::map<std::string, geometry_msgs::msg::Point>& all_positions,
        std::string my_id) 
    {
        // TUNING PARAMETERS
        double safe_radius = 0.7; // Meters (Trigger distance)
        double gain = 1.2;        // Strength of the push
        
        double dx_rep = 0.0;
        double dy_rep = 0.0;
        bool avoidance_active = false;

        for (const auto& [other_id, other_pos] : all_positions) {
            if (other_id == my_id) continue; // Don't repel from self

            double dist = std::hypot(current.x - other_pos.x, current.y - other_pos.y);
            
            // Avoid division by zero
            if (dist < 0.05) dist = 0.05; 

            if (dist < safe_radius) {
                avoidance_active = true;
                
                // Vector FROM other drone TO me (Push Away)
                double dx = current.x - other_pos.x;
                double dy = current.y - other_pos.y;
                
                // Formula: Strength increases as distance decreases
                // Normalized Vector (dx/dist) * Magnitude (gain * (safe - dist))
                double strength = gain * (safe_radius - dist) / dist;
                
                dx_rep += dx * strength;
                dy_rep += dy * strength;
            }
        }

        if (avoidance_active) {
            target.x += dx_rep;
            target.y += dy_rep;
        }
        
        return target;
    }

    void publishCommands() {
        struct DroneCmd {
            std::string id;
            std::vector<crazyflie_interfaces::msg::Position> waypoints;
        };
        std::vector<DroneCmd> batch_buffer;
        
        // NEW: We need a snapshot of where everyone is RIGHT NOW
        std::map<std::string, geometry_msgs::msg::Point> current_positions;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (current_state_ != SwarmState::MISSION) return;

            // Snapshot Commands
            batch_buffer.reserve(active_commands_.size());
            for (const auto& kv : active_commands_) {
                if (!kv.second.empty()) {
                    batch_buffer.push_back({kv.first, kv.second});
                }
            }
            
            // Snapshot Positions (for collision checks)
            for(const auto& kv : swarm_poses_) {
                current_positions[kv.first] = kv.second.pose.position;
            }
        } 

        #pragma omp parallel for
        for (int i = 0; i < (int)batch_buffer.size(); ++i) {
            const auto& item = batch_buffer[i];
            
            auto pub_it = cmd_pubs_.find(item.id);
            
            // Only proceed if we have a publisher and a current position for this drone
            if (pub_it != cmd_pubs_.end() && !item.waypoints.empty() && current_positions.count(item.id)) {
                
                // 1. Pick the Next Waypoint
                size_t target_idx = (item.waypoints.size() > 1) ? 1 : 0;
                auto next_wp = item.waypoints[target_idx];
                
                // Convert to Point for calculation
                geometry_msgs::msg::Point target_pt;
                target_pt.x = next_wp.x;
                target_pt.y = next_wp.y;
                target_pt.z = next_wp.z; // Repulsion usually purely horizontal (XY)

                // 2. APPLY REPULSION
                // Nudge the target point away from neighbors
                geometry_msgs::msg::Point adjusted_pt = applyRepulsion(
                    target_pt, 
                    current_positions[item.id], 
                    current_positions, 
                    item.id
                );

                // 3. Publish the Adjusted Command
                crazyflie_interfaces::msg::Position safe_cmd = next_wp; // Copy yaw/z
                safe_cmd.x = adjusted_pt.x;
                safe_cmd.y = adjusted_pt.y;
                
                pub_it->second->publish(safe_cmd);
            }
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SwarmPlanner>();
    
    // Critical for concurrency: Allows Subscriber to run while Timer is planning
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}