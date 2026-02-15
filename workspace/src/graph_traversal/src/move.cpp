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
#include <atomic>
#include <mutex>
#include <queue>
#include <map>
#include <unordered_set>
#include <omp.h>

#include <sensor_msgs/msg/battery_state.hpp>
#include <crazyflie_interfaces/srv/land.hpp>
#include "utils.hpp"
#include <std_msgs/msg/float64.hpp>

using namespace std::chrono_literals;

struct idx { int i, j; };

struct DroneBatteryState {
    float percentage = 100.0f;
    bool has_charged = false;     // Can only charge once
    bool is_charging = false;     // Currently on ground charging
    bool is_going_to_charge = false; // En route to charging area
    bool is_leaving_charger = false; // NEW: Post-charge clearance maneuver
    rclcpp::Time start_leaving_time; // NEW: Timer for clearance
    int assigned_slot = -1;       // -1: None, 0..N: Slot Index
    float start_charge_percentage = 0.0f; // Battery level when charging started
};

// Node for A* Priority Queue
struct AStarNode {
    int idx_flat;
    int f_score; 
    bool operator>(const AStarNode& other) const { return f_score > other.f_score; }
};

enum class SwarmState { TAKEOFF, MISSION, RETURN_TO_HOME };

class SwarmPlanner : public rclcpp::Node {
public:
    SwarmPlanner() : Node("Swarm_planner") {
        this->declare_parameter<double>("inflation_radius", 0.3); // Reduced slightly for better fit
        this->declare_parameter<double>("z_target", 1.0);
        current_state_ = SwarmState::TAKEOFF;
        int N = get_num_robots();
        for (int i=1;i<=N;i++){
            droneIds_.push_back("cf_"+std::to_string(i));
        }

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
            }, sub_opt);

        // 1b. z_target from bfs (height changes on AGV loop)
        z_target_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/mission/z_target", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                z_target_from_bfs_.store(msg->data);
                has_z_target_ = true;
            }, sub_opt);

        // 2. Drone State Subscriptions
        for (const std::string& id : droneIds_) {
            drone_subs_.push_back(this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/" + id + "/pose", 10,
                [this, id](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    this->swarm_poses_[id] = *msg;
                    // Capture Initial Pose
                    if (this->initial_poses_.find(id) == this->initial_poses_.end()) {
                        this->initial_poses_[id] = msg->pose.position;
                        RCLCPP_INFO(this->get_logger(), "🏠 Captured Home Position for %s: (%.2f, %.2f, %.2f)", 
                            id.c_str(), msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
                    }
                }, sub_opt));

            cmd_pubs_[id] = this->create_publisher<crazyflie_interfaces::msg::Position>(
                "/" + id + "/cmd_position", 10);
        }

        // AGV Pose Subscription for loop detection - DISABLED FOR NOW
        
        agv_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/AGV/pose", 10,
            [this](const geometry_msgs::msg::Point::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                agv_current_pos_ = *msg;
                
                // Capture starting position once
                if (!agv_start_captured_) {
                    agv_start_pos_ = *msg;
                    agv_start_captured_ = true;
                    RCLCPP_INFO(this->get_logger(), "AGV start position captured: (%.2f, %.2f)", msg->x, msg->y);
                }
            }, sub_opt);
        

        // 3. Clients
        octomap_client_ = this->create_client<octomap_msgs::srv::GetOctomap>("octomap_binary", 
            rmw_qos_profile_services_default, callback_group_);
        


        // 4. Timers
        // Init: Get map once
        init_timer_ = this->create_wall_timer(500ms, [this]() { if (!map_ready_) fetchOctomapOnce(); });
        
        // Planning Loop (High Level Logic)
        timer_ = this->create_wall_timer(100ms, std::bind(&SwarmPlanner::runSwarmSystem, this), callback_group_);
        
        // Control Loop (Publishing commands fast)
        control_timer_ = this->create_wall_timer(50ms, std::bind(&SwarmPlanner::publishCommands, this), callback_group_);
        
        initializeBMS();
    }

private:
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::TimerBase::SharedPtr timer_, init_timer_, control_timer_;
    rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedPtr octomap_client_;
    
    SwarmState current_state_;
    bool takeoff_called_ = false;
    const double REACH_THRESHOLD = 0.15;
    
    std::mutex data_mutex_;
    std::unique_ptr<octomap::OcTree> tree_;
    nav_msgs::msg::OccupancyGrid cached_grid_;
    geometry_msgs::msg::PoseArray current_bfsPoints_;
    bool hasPoints_ = false;
    bool map_ready_ = false;

    // Height boost after AGV loop completion
    const double HEIGHT_BOOST = 1.0;        // 1 meter increase per loop
    const double LOOP_DETECTION_RADIUS = 1.0; // meters - threshold to detect AGV returning to start
    int agv_loop_count_ = 0;
    geometry_msgs::msg::Point agv_start_pos_;
    geometry_msgs::msg::Point agv_current_pos_;
    bool agv_start_captured_ = false;
    bool agv_away_from_start_ = false;  // Must move away before loop can be counted
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr agv_sub_;

    std::vector<std::string> droneIds_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> drone_subs_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr dronePos_sub_;
    std::atomic<double> z_target_from_bfs_{1.0};
    std::atomic<bool> has_z_target_{false};
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr z_target_sub_;

    std::map<std::string, geometry_msgs::msg::PoseStamped> swarm_poses_;
    std::map<std::string, rclcpp::Publisher<crazyflie_interfaces::msg::Position>::SharedPtr> cmd_pubs_;
    std::map<std::string, std::vector<crazyflie_interfaces::msg::Position>> active_commands_;
    std::map<std::string, geometry_msgs::msg::Point> initial_poses_; // NEW: Store start positions
    int rth_index_ = 0; // NEW: Track which drone is returning


    // --- BMS Members ---
    std::map<std::string, DroneBatteryState> battery_states_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr> battery_subs_;
    std::map<std::string, rclcpp::Client<crazyflie_interfaces::srv::Land>::SharedPtr> land_clients_;
    std::map<std::string, rclcpp::Client<crazyflie_interfaces::srv::Takeoff>::SharedPtr> takeoff_clients_;
    
    geometry_msgs::msg::Point charging_station_coords_;
    const float CHARGING_THRESHOLD = 70.0f; // Updated per user desire/code snapshot
    const float CHARGED_THRESHOLD = 88.0f;

    // --- Charging Slots & Concurrency ---
    std::vector<geometry_msgs::msg::Point> charging_slots_;
    std::vector<bool> slot_occupied_;
    const int MAX_CHARGING_DRONES = 2; // User limit

    void initializeBMS() {
        // 1. Load Charging Area Parameters
        std::vector<double> ul = charging_area_upper_left();
        std::vector<double> dr = charging_area_down_right();
        
        // Compute Center
        double cx = 0.0, cy = 0.0;
        if (ul.size() >= 2 && dr.size() >= 2) {
            cx = (ul[0] + dr[0]) / 2.0;
            cy = (ul[1] + dr[1]) / 2.0;
            charging_station_coords_.x = cx;
            charging_station_coords_.y = cy;
            charging_station_coords_.z = 0.0;
        } else {
             RCLCPP_ERROR(this->get_logger(), "Invalid charging area parameters! Defaulting to 0,0");
        }

        // Initialize Slots (Diagonally opposite corners for max separation)
        // Offset slightly inward from corners to stay within bounds? 
        // Or just use the corners if safe. defined as [-1,1] to [1,-1].
        // Let's go 0.5m inward from corners to be safe, or just fixed offsets from center if area is large enough.
        // Given area is likely large enough, let's use fixed offsets from center to ensure >1.4m separation.
        // Slot 0: Center + (-0.8, -0.8)
        // Slot 1: Center + (0.8, 0.8)
        // Distance ~ 2.2m > 1.4m required.
        
        geometry_msgs::msg::Point s0; s0.x = cx - 0.8; s0.y = cy - 0.8; s0.z = 0.0;
        geometry_msgs::msg::Point s1; s1.x = cx + 0.8; s1.y = cy + 0.8; s1.z = 0.0;
        
        charging_slots_.push_back(s0);
        charging_slots_.push_back(s1);
        slot_occupied_.resize(2, false);

        // 2. Subscribe and Create Clients
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_;

        for (const std::string& id : droneIds_) {
            // Init State
            battery_states_[id] = DroneBatteryState();

            // Battery Sub
            battery_subs_.push_back(this->create_subscription<sensor_msgs::msg::BatteryState>(
                "/" + id + "/battery_status", 10,
                [this, id](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    this->battery_states_[id].percentage = msg->percentage;
                }, sub_opt));

            // Land Client
            land_clients_[id] = this->create_client<crazyflie_interfaces::srv::Land>(
                "/" + id + "/land", rmw_qos_profile_services_default, callback_group_);
                
            // Takeoff Client (Individual)
            takeoff_clients_[id] = this->create_client<crazyflie_interfaces::srv::Takeoff>(
                "/" + id + "/takeoff", rmw_qos_profile_services_default, callback_group_);
        }
    }

    // --- DISTANCE HELPER FUNCTIONS ---
    // 2D horizontal distance (for drone-to-drone, same altitude)
    double dist2D(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
        return std::hypot(a.x - b.x, a.y - b.y);
    }
    
    // 3D Euclidean distance (for drone-to-AGV/base, different altitudes)
    double dist3D(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        double dz = a.z - b.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }

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

    std::vector<geometry_msgs::msg::Point> runPlanningPipeline(const nav_msgs::msg::OccupancyGrid& map, geometry_msgs::msg::Point target, geometry_msgs::msg::Point current_pos, double target_z) {
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
        } else if (current_state_ == SwarmState::RETURN_TO_HOME) {
            handleReturnToHome();
        }
    }

    void handleBatteryLogic(const std::string& id) {
        auto& state = battery_states_[id];
        auto current_pos = swarm_poses_[id].pose.position;

        // 3. Check for Takeoff (Charged -> Takeoff & Clear)
        // 3. Check for Takeoff (Charged -> Takeoff & Clear)
        if (state.is_charging) {
            // Check if charged
            if (state.percentage >= 88.0) { // 88% Threshold
                state.is_charging = false;
                state.is_leaving_charger = true;
                state.start_leaving_time = this->now();
                
                // DO NOT FREE SLOT YET - WAIT UNTIL CLEARED!
                // state.assigned_slot remains valid
                
                RCLCPP_INFO(this->get_logger(), "🔋 Drone %s CHARGED. Low Takeoff (Z=0.5) & Clearing Area.", id.c_str());
            }
            return;
        }

        // 2. Check for Landing (Going -> Land)
        if (state.is_going_to_charge) {
            // Target is their ASSIGNED SLOT, not center
            if (state.assigned_slot < 0 || state.assigned_slot >= (int)charging_slots_.size()) return;
            
            geometry_msgs::msg::Point target = charging_slots_[state.assigned_slot];
            double dist = std::hypot(current_pos.x - target.x, current_pos.y - target.y);
            
            if (dist < 0.1 && !state.is_charging) {
                RCLCPP_INFO(this->get_logger(), "⬇️ Drone %s arrived at charging slot %d. 'Landing' to Z=0.05.", id.c_str(), state.assigned_slot);
                
                // NO LAND SERVICE - Just state switch + Z command
                state.is_charging = true;
                state.is_going_to_charge = false;
            }
        }
    }

    void handleTakeoff() {
        if (!takeoff_called_) {
            for(const auto& id : droneIds_) {
                auto client = takeoff_clients_[id];
                if (!client->service_is_ready()) continue;
                
                auto req = std::make_shared<crazyflie_interfaces::srv::Takeoff::Request>();
                req->height = 1.0; 
                req->duration.sec = 2;
                req->duration.nanosec = 0;
                client->async_send_request(req);
            }
            takeoff_called_ = true;
            return;
        }

        bool all_up = true;
        double z_target = has_z_target_ ? z_target_from_bfs_.load() : this->get_parameter("z_target").as_double();
        
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
        // AGV Loop Detection - DISABLED FOR NOW
        
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (agv_start_captured_) {
                double dist_to_start = std::hypot(
                    agv_current_pos_.x - agv_start_pos_.x,
                    agv_current_pos_.y - agv_start_pos_.y
                );
                
                // Must move away first (at least 3m from start)
                if (!agv_away_from_start_ && dist_to_start > get_comm_range()) {
                    agv_away_from_start_ = true;
                    RCLCPP_INFO(this->get_logger(), "AGV moved away from start, loop detection active");
                }
                
                // Detect loop completion - AGV returned to start
                if (agv_away_from_start_ && dist_to_start < LOOP_DETECTION_RADIUS) {
                    agv_loop_count_++;
                    agv_away_from_start_ = false;  // Reset for next loop
                    
                    double current_z = this->get_parameter("z_target").as_double();
                    this->set_parameter(rclcpp::Parameter("z_target", current_z + HEIGHT_BOOST));
                    RCLCPP_INFO(this->get_logger(), 
                        "🔄 AGV Loop %d completed! Raising altitude to %.1f m", 
                        agv_loop_count_, current_z + HEIGHT_BOOST);
                }
            }
        }
        

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

        // z_mission from /mission/z_target topic (bfs publishes, updates on AGV loop)
        const double z_mission = has_z_target_ ? z_target_from_bfs_.load() : this->get_parameter("z_target").as_double();

        const size_t num_drones = droneIds_.size();
        const size_t num_targets = local_targets.poses.size();


        // --- BMS: SEPARATE AVAILABLE VS UNAVAILABLE DRONES ---
        std::vector<std::string> available_drones;
        std::vector<std::string> charging_drones;

        {
             std::lock_guard<std::mutex> lock(data_mutex_);
             for(const auto& id : droneIds_) {
                 handleBatteryLogic(id); // Update internal states
                 
                 // 1. Check if ALREADY charging/going/leaving
                 if (battery_states_[id].is_charging || battery_states_[id].is_going_to_charge || battery_states_[id].is_leaving_charger) {
                     charging_drones.push_back(id);
                 } 
                 else {
                     // 2. CHECK IF NEEDS CHARGE (Hot Swap Trigger)
                     // Even if assigned, if battery is low and slot available -> Send to charge
                     bool condition_met = false;
                     
                     if (battery_states_[id].percentage < CHARGING_THRESHOLD && !battery_states_[id].has_charged) {
                         // Check Concurrency
                         int currently_charging_count = 0;
                         for(const auto& b : battery_states_) {
                            if (b.second.is_charging || b.second.is_going_to_charge) currently_charging_count++;
                         }
                         
                         // Check Slot
                         int free_slot = -1;
                         for(size_t s=0; s<slot_occupied_.size(); ++s) {
                            if(!slot_occupied_[s]) { free_slot = s; break; }
                         }

                         if (currently_charging_count < MAX_CHARGING_DRONES && free_slot != -1) {
                             // TRIGGER CHARGE
                             battery_states_[id].is_going_to_charge = true;
                             battery_states_[id].assigned_slot = free_slot;
                             battery_states_[id].start_charge_percentage = battery_states_[id].percentage;
                             slot_occupied_[free_slot] = true;
                             RCLCPP_WARN(this->get_logger(), "🪫 Drone %s Low Battery (%.1f%%). Sending to Charge Slot %d.", 
                                id.c_str(), battery_states_[id].percentage, free_slot);
                             
                             charging_drones.push_back(id);
                             condition_met = true;
                         }
                     }
                     
                     if (!condition_met) {
                         // User Feedback 1: Continue mission if queues are full
                         // Don't freeze. Just keep them available.
                         available_drones.push_back(id);
                     }
                 }
             }

             // --- PRIORITY CHECK: RECALL IF NEEDED (SMART RECALL) ---
             // Sort charging_drones by Battery Percentage (Highest First)
             std::sort(charging_drones.begin(), charging_drones.end(), 
                 [this](const std::string& a, const std::string& b) {
                     return battery_states_[a].percentage > battery_states_[b].percentage;
                 });

             // If we have more targets than available drones, recall BEST candidates
             while (available_drones.size() < num_targets && !charging_drones.empty()) {
                 // Pick Best Candidate (Highest Battery) - now at beginning after sort? 
                 // wait, std::sort defaults to ascending. We want descending (Highest first).
                 // So "percentage > percentage" means a > b. 
                 // So beginning is Highest.
                 
                 std::string recall_id = charging_drones.front(); // BEST BATTERY
                 
                 // --- GUARD CONDITION: RECALL THRESHOLD ---
                 // Only recall if we have gained at least 10% charge OR if we have NO available drones
                 float current_pct = battery_states_[recall_id].percentage;
                 float start_pct = battery_states_[recall_id].start_charge_percentage;
                 
                 bool can_recall = (current_pct >= start_pct + 10.0f) || (available_drones.empty());
                 
                 if (!can_recall && !available_drones.empty()) {
                     // We have SOME drones, and this charging drone hasn't charged enough.
                     // Skip it and stop looking (since this was the BEST one).
                     break; 
                 }
                 
                 // Proceed with recall
                 charging_drones.erase(charging_drones.begin());
                 
                 available_drones.push_back(recall_id);
                 
                 // Reset State
                 auto& state = battery_states_[recall_id];
                 
                 // If was on ground charging, TAKE OFF (Virtual)
                 if (state.is_charging) {
                    RCLCPP_WARN(this->get_logger(), "🚨 RECALLING Drone %s (%.1f%%) from charging for MISSION!", 
                        recall_id.c_str(), state.percentage);
                    // No Service call needed, just state change.
                    // Loop will pick up Z=Mission Height next cycle.
                 } else {
                    RCLCPP_INFO(this->get_logger(), "↩️ Redirecting Drone %s (%.1f%%) from charging approach to MISSION.", 
                        recall_id.c_str(), state.percentage);
                 }

                 // Free Slot
                 if (state.assigned_slot >= 0 && state.assigned_slot < (int)slot_occupied_.size()) {
                    slot_occupied_[state.assigned_slot] = false;
                 }
                 state.assigned_slot = -1;
                 state.is_charging = false;
                 state.is_going_to_charge = false;
                 state.has_charged = true; 
             }
        }

        // --- GLOBAL BATTERY EMERGENCY CHECK ---
        bool emergency_rth = false;
        {
             std::lock_guard<std::mutex> lock(data_mutex_);
             for(const auto& id : droneIds_) {
                 if (battery_states_[id].percentage < 25.0) {
                     emergency_rth = true;
                     RCLCPP_ERROR(this->get_logger(), "🚨 EMERGENCY RTH TRIGGERED! Drone %s Battery at %.1f%% (< 25%%)", 
                         id.c_str(), battery_states_[id].percentage);
                 }
             }
        }

        if (emergency_rth) {
            current_state_ = SwarmState::RETURN_TO_HOME;
            return; // Switch immediately
        }
        
        const size_t num_avail = available_drones.size();
        
        // --- 1. Matching Logic (Cost Matrix & Binary Search) ---
        // ONLY match AVAILABLE drones
        std::vector<std::vector<double>> costs(num_avail, std::vector<double>(num_targets));
        std::vector<double> all_distances;

        for (size_t i = 0; i < num_avail; ++i) {
            for (size_t j = 0; j < num_targets; ++j) {
                auto& d_pos = local_poses[available_drones[i]].pose.position;
                auto& t_pos = local_targets.poses[j].position;
                // Use 3D distance for drone-to-target (different altitudes)
                double dist = dist3D(d_pos, t_pos);
                costs[i][j] = dist;
                all_distances.push_back(dist);
            }
        }
        std::sort(all_distances.begin(), all_distances.end());

        int low = 0, high = all_distances.size() - 1;
        std::vector<int> final_match(num_targets, -1);
        double best_threshold = (all_distances.empty()) ? 0.0 : all_distances.back();
        int required_matches = std::min(num_avail, num_targets);

        while (low <= high) {
            int mid = low + (high - low) / 2;
            double threshold = all_distances[mid];
            std::vector<int> current_match(num_targets, -1);
            int matches_found = 0;
            
            for (size_t i = 0; i < num_avail; ++i) {
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

        std::vector<DroneJob> jobs(num_drones); // We still need entries for ALL drones to process them
        // But we only fill "jobs" based on available list + special handling for charging

        
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

        // A. Process AVAILABLE DRONES
        for (size_t i = 0; i < num_avail; ++i) {
            // Find which global index this drone is
            std::string id = available_drones[i];
            
            // Find global index for "jobs" array
            auto it = std::find(droneIds_.begin(), droneIds_.end(), id);
            size_t global_idx = std::distance(droneIds_.begin(), it);
            
            jobs[global_idx].id = id;
            auto current_pos = local_poses[id].pose.position;

            if (drone_to_target.count(i)) {
                // --- ACTIVE DRONE ---
                jobs[global_idx].is_assigned = true;
                int t_idx = drone_to_target[i];
                jobs[global_idx].target_pos = local_targets.poses[t_idx].position;
                jobs[global_idx].target_z = z_mission;
            } else {
                // --- IDLE DRONE (SHADOW STRATEGY) ---
                jobs[global_idx].is_assigned = false;
                
                // NO BATTERY CHECK HERE - ALREADY HANDLED IN PRE-MATCHING!
                // Just fallback to Shadow Logic

                // Standard Shadow Logic
                int best_anchor_idx = 0;
                double min_dist = 1e9;
                for (size_t k = 0; k < anchors.size(); ++k) {
                    double d = dist3D(current_pos, anchors[k]);
                    if (d < min_dist) { min_dist = d; best_anchor_idx = k; }
                }
                geometry_msgs::msg::Point shadow_target = anchors[best_anchor_idx];
                double offset_dist = 2.0;

                // Hashing for consistent offset
                int magic = id[3] - '0'; // cf_X
                if (magic % 4 == 0)      shadow_target.x += offset_dist;
                else if (magic % 4 == 1) shadow_target.x -= offset_dist;
                else if (magic % 4 == 2) shadow_target.y += offset_dist;
                else                     shadow_target.y -= offset_dist;

                jobs[global_idx].target_pos = shadow_target;
                jobs[global_idx].target_z = z_mission; 
            }
        }
        
        // B. Process CHARGING DRONES (includes Going, Charging, Leaving)
        for (const std::string& id : charging_drones) {
            auto it = std::find(droneIds_.begin(), droneIds_.end(), id);
            size_t global_idx = std::distance(droneIds_.begin(), it);
            jobs[global_idx].id = id;
            jobs[global_idx].is_assigned = true; 
            
            // 1. CHECK IF LEAVING CHARGER
            if (battery_states_[id].is_leaving_charger) {
                 double elapsed = (this->now() - battery_states_[id].start_leaving_time).seconds();
                 
                 // Distance check for early release
                 double dx = local_poses[id].pose.position.x - charging_station_coords_.x;
                 double dy = local_poses[id].pose.position.y - charging_station_coords_.y;
                 double dist_from_center = std::hypot(dx, dy);

                 // User Feedback 2: Don't get stuck. Clear if moved away (>1.5m) OR timeout
                 if (dist_from_center > 1.5 || elapsed > 5.0) {
                     // Finished clearing
                     {
                         std::lock_guard<std::mutex> lock(data_mutex_);
                         battery_states_[id].is_leaving_charger = false;
                         battery_states_[id].has_charged = true; 
                         
                         // LATE SLOT RELEASE:
                         int s = battery_states_[id].assigned_slot;
                         if (s >= 0 && s < (int)slot_occupied_.size()) {
                             slot_occupied_[s] = false;
                             RCLCPP_INFO(this->get_logger(), "🕊️ Slot %d freed by %s (Cleared dist=%.2fm)", s, id.c_str(), dist_from_center);
                         }
                         battery_states_[id].assigned_slot = -1;
                     }
                     // Drone will become available next cycle
                     RCLCPP_INFO(this->get_logger(), "✅ Drone %s Cleared Charging Area.", id.c_str());
                     
                     // Current cycle: Just Hover where it is (or move to safe spot)
                     // It's effectively free now.
                 } else {
                     // EXECUTING CLEARANCE MANEUVER
                     // Move horizontally away from center
                     double dx = local_poses[id].pose.position.x - charging_station_coords_.x;
                     double dy = local_poses[id].pose.position.y - charging_station_coords_.y;
                     
                     // Normalized vector away from center
                     double dist_c = std::hypot(dx, dy);
                     double nx = (dist_c > 0.01) ? dx/dist_c : 1.0;
                     double ny = (dist_c > 0.01) ? dy/dist_c : 0.0;
                     
                     // Target = Center + 2.0m direction
                     jobs[global_idx].target_pos.x = charging_station_coords_.x + nx * 2.0;
                     jobs[global_idx].target_pos.y = charging_station_coords_.y + ny * 2.0;
                     jobs[global_idx].target_pos.z = 0.5; // Keep LOW
                     jobs[global_idx].target_z = 0.5;
                 }
                 continue; // Done with this drone
            }

            // 2. CHECK IF GOING TO CHARGE OR CHARGING (Landed)
            // Get Assigned Slot
            int slot = battery_states_[id].assigned_slot;
            if (slot >= 0 && slot < (int)charging_slots_.size()) {
                jobs[global_idx].target_pos = charging_slots_[slot];
            } else {
                // ERROR STATE: Should have slot. Hover safety.
                RCLCPP_ERROR(this->get_logger(), "Drone %s has invalid slot %d! Hovering.", id.c_str(), slot);
                jobs[global_idx].target_pos = local_poses[id].pose.position;
                jobs[global_idx].target_z = z_mission;
                continue;
            }
            
            // If already charging (landed), set low Z.
            if (battery_states_[id].is_charging) {
                 // HOLD POSITION ON SLOT AT LOW Z
                 jobs[global_idx].target_z = 0.05; 
            } else {
                 // Going to charge (Transit) -> APPROACH HEIGHT 0.5
                 // Safe but distinct from ground
                 jobs[global_idx].target_z = 0.5; 
            }
        }

        // C. Process WAITING DRONES - REMOVED PER FEEDBACK
        // (Drones in queue are now in Available list)

        std::vector<std::pair<std::string, std::vector<crazyflie_interfaces::msg::Position>>> new_commands(droneIds_.size());

        // Compute paths for assigned jobs
        for (int i = 0; i < (int)droneIds_.size(); ++i) {
            std::string id = droneIds_[i];
            
            // NOTE: We now generate commands for EVERYONE, including charging drones (to hold position)
            // So no continue skip here.
            
            DroneJob& job = jobs[i];
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
                 // Run A* Pipeline with explicit Z
                 auto path = runPlanningPipeline(local_grid, job.target_pos, current_pos, job.target_z);
                 
                 // Smoothing / Post-Processing (omitted for brevity, assume path is usable)
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

    void handleReturnToHome() {
        // Sequential Logic: One by one return
        
        nav_msgs::msg::OccupancyGrid local_grid;
        std::map<std::string, geometry_msgs::msg::PoseStamped> local_poses;
        std::map<std::string, geometry_msgs::msg::Point> home_poses;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            local_grid = cached_grid_;
            local_poses = swarm_poses_;
            home_poses = initial_poses_;
        }

        std::map<std::string, std::vector<crazyflie_interfaces::msg::Position>> new_commands;
        double z_safe = 1.0; // Fly home at safe height

        // 1. Generate Commands
        #pragma omp parallel for
        for (int i = 0; i < (int)droneIds_.size(); ++i) {
            std::string id = droneIds_[i];
            
            if (!home_poses.count(id)) continue; 
            
            auto current_pos = local_poses[id].pose.position;
            auto home_pos = home_poses[id];
            
            std::vector<crazyflie_interfaces::msg::Position> cmd_list;

            // --- SEQUENTIAL LOGIC ---
            if (i < rth_index_) {
                // ALREADY LANDED -> STAY DOWN
                crazyflie_interfaces::msg::Position cmd;
                cmd.x = home_pos.x; cmd.y = home_pos.y; cmd.z = 0.05; cmd.yaw = 0.0;
                cmd_list.push_back(cmd);
            } 
            else if (i > rth_index_) {
                 // WAITING -> HOVER IN PLACE
                 crazyflie_interfaces::msg::Position cmd;
                 cmd.x = current_pos.x; cmd.y = current_pos.y; cmd.z = current_pos.z; cmd.yaw = 0.0;
                 cmd_list.push_back(cmd);
            }
            else {
                // ACTIVE DRONE (i == rth_index_) -> GO HOME
                
                // Check distance to home (2D)
                double dist = std::hypot(current_pos.x - home_pos.x, current_pos.y - home_pos.y);
                
                if (dist < 0.2) {
                    // ARRIVED AT HOME -> LAND
                    crazyflie_interfaces::msg::Position cmd;
                    cmd.x = home_pos.x;
                    cmd.y = home_pos.y;
                    cmd.z = 0.05; // Land
                    cmd.yaw = 0.0;
                    cmd_list.push_back(cmd);
                    
                    // CHECK IF ACTUALLY LANDED (Z < 0.1)
                    // We need to check this in the main thread (not parallel) or accept race condition?
                    // Actually, we can just check here, but we can't increment rth_index_ inside parallel region easily without atomic or critical.
                    // But since we only care about THIS drone (i == rth_index_), only one thread will hit this.
                    
                    if (current_pos.z < 0.15) {
                        // Drone has landed. Move to next!
                        // We'll update rth_index_ in the critical section below or separate step.
                        // Let's mark it for update.
                        #pragma omp critical
                        {
                            // Only if I am the current one (double check)
                            if (i == rth_index_) {
                                rth_index_++;
                                RCLCPP_INFO(this->get_logger(), "✅ Drone %s Landed. Next!", id.c_str());
                            }
                        }
                    }
                } else {
                    // NAVIGATE TO HOME
                    auto path = runPlanningPipeline(local_grid, home_pos, current_pos, z_safe);
                    
                    if (!path.empty()) {
                        for (size_t k = 0; k < path.size(); ++k) {
                            crazyflie_interfaces::msg::Position cmd;
                            cmd.x = path[k].x; cmd.y = path[k].y; cmd.z = z_safe;
                            
                            double dx, dy;
                            if (k == 0) { dx = path[k].x - current_pos.x; dy = path[k].y - current_pos.y; }
                            else { dx = path[k].x - path[k-1].x; dy = path[k].y - path[k-1].y; }
                            
                            if (std::hypot(dx, dy) > 0.01) cmd.yaw = std::atan2(dy, dx);
                            else cmd.yaw = (cmd_list.empty()) ? 0.0 : cmd_list.back().yaw;
                            
                            cmd_list.push_back(cmd);
                        }
                    } else {
                        // Fallback: Hold current position
                        crazyflie_interfaces::msg::Position cmd;
                        cmd.x = current_pos.x; cmd.y = current_pos.y; cmd.z = z_safe; cmd.yaw = 0.0;
                        cmd_list.push_back(cmd);
                    }
                }
            }
            
            #pragma omp critical
            new_commands[id] = cmd_list;
        }

        // 2. Commit
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            active_commands_ = new_commands;
        }
    }

    geometry_msgs::msg::Point applyRepulsion(
        geometry_msgs::msg::Point target,
        geometry_msgs::msg::Point current,
        const std::map<std::string, geometry_msgs::msg::Point>& all_positions,
        std::string my_id) 
    {
        // TUNING PARAMETERS
        // Dynamic Safe Radius: Standard 0.7m, but reduced during RTH to allow landing at close start points
        double safe_radius = (current_state_ == SwarmState::RETURN_TO_HOME) ? 0.2 : 0.7; 
        
        double gain = 1.2;        // Strength of the push
        const double MAX_REPULSION_STEP = 1.0; // CLAMP: Max 1m shift per step

        double dx_rep = 0.0;
        double dy_rep = 0.0;
        bool avoidance_active = false;

        for (const auto& [other_id, other_pos] : all_positions) {
            if (other_id == my_id) continue; // Don't repel from self

            // Use 2D horizontal distance for drone-to-drone (same altitude)
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
            // CLAMPING LOGIC
            double rep_mag = std::hypot(dx_rep, dy_rep);
            if (rep_mag > MAX_REPULSION_STEP) {
                dx_rep = (dx_rep / rep_mag) * MAX_REPULSION_STEP;
                dy_rep = (dy_rep / rep_mag) * MAX_REPULSION_STEP;
            }

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
            if (current_state_ != SwarmState::MISSION && current_state_ != SwarmState::RETURN_TO_HOME) return;

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
                safe_cmd.z = adjusted_pt.z; // Use clamped/repulsed Z
                
                pub_it->second->publish(safe_cmd);
            }
        }
    } // end publishCommands
}; // end SwarmPlanner

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
