#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <crazyflie_interfaces/msg/position.hpp>
#include <icuas25_msgs/msg/target_info.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <queue>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <vector>


#include <tuple>

static const std::map<int, std::tuple<double, double, double>> GROUND_TRUTH{
    {11, {-5.9245, -1.3903, 2.0560}}, {14, {7.6041, 2.8089, 2.3750}},
    {16, {-3.8641, 6.9430, 1.7940}},  {19, {4.8932, -4.3532, 1.7024}},
    {15, {8.1615, 8.7107, 1.9291}},   {13, {4.8295, 1.5269, 1.0629}},
    {18, {-2.6701, 7.2078, 2.9439}},  {20, {-7.6504, -8.5481, 2.3014}},
    {17, {1.2507, -6.5897, 1.7486}},  {4, {0.5277, -4.3226, 3.1164}},
    {29, {7.6705, -5.2531, 2.0552}},  {21, {-7.4942, -9.0051, 2.9806}},
    {39, {-7.7655, -9.0323, 0.8734}}, {24, {-1.9260, -2.1796, 0.9641}},
    {38, {-1.7333, -4.9403, 2.1630}}, {9, {-2.0129, -2.0421, 1.9796}},
    {6, {0.4546, -3.8881, 1.0082}},   {36, {6.9151, -8.2716, 1.3943}},
    {35, {-5.8848, -1.2585, 0.6699}}, {8, {5.9808, 5.9191, 1.8864}},
    {25, {6.5083, -2.1766, 1.0218}},  {37, {-1.2361, -4.8519, 1.2981}},
    {2, {-2.2373, 1.9367, 0.9951}},   {32, {6.0451, 6.4071, 1.0640}},
    {26, {-2.1589, 7.1959, 1.2590}},  {22, {-8.6736, 4.2784, 0.5363}},
    {7, {0.7506, -3.8684, 2.2749}},   {27, {-8.8289, 4.4761, 1.4376}},
};

static int get_num_robots() {
    const char* env = std::getenv("NUM_ROBOTS");
    return env ? std::stoi(env) : 5;
}

static double get_comm_range() {
    const char* env = std::getenv("COMM_RANGE");
    return env ? std::stod(env) : 70.0;
}

static std::string get_charging_file() {
    const char* env = std::getenv("CHARGING_FILE");
    return env ? std::string(env) : "5";
}


class DebugLoggerNode : public rclcpp::Node {
public:
    DebugLoggerNode() : Node("debug_logger") {
        
        num_robots_ = get_num_robots();
        comm_range_ = get_comm_range();

        for (int i = 1; i <= num_robots_; ++i)
            drone_ids_.push_back("cf_" + std::to_string(i));

        
        try {
            std::string config_path = "/root/ros2_ws/src/icuas26_competition/config/";
            YAML::Node config = YAML::LoadFile(config_path + get_charging_file());
            charging_ul_ = {config["charging_area"]["upper_left"][0].as<double>(),
                            config["charging_area"]["upper_left"][1].as<double>()};
            charging_dr_ = {config["charging_area"]["down_right"][0].as<double>(),
                            config["charging_area"]["down_right"][1].as<double>()};
            has_charging_area_ = true;
            RCLCPP_INFO(get_logger(), "Charging area loaded: UL(%.1f,%.1f) DR(%.1f,%.1f)",
                        charging_ul_[0], charging_ul_[1], charging_dr_[0], charging_dr_[1]);
        } catch (...) {
            RCLCPP_WARN(get_logger(), "Could not load charging area config – some events will be skipped");
        }

        //environment bounds
        env_min_x_ = -15.0;  env_min_y_ = -15.0;
        env_max_x_ =  15.0;  env_max_y_ =  15.0;
        env_max_z_ =  10.0;

        
        //open log file
        log_file_.open("/root/workspace/log/mission_log.txt", std::ios::out | std::ios::trunc);
        if (log_file_.is_open()) {
            RCLCPP_INFO(get_logger(), "Mission log file: /root/workspace/log/mission_log.txt");
        }


        auto sub_opt = rclcpp::SubscriptionOptions();

        // Per-drone topics
        for (const auto& id : drone_ids_) {
            // Pose
            pose_subs_.push_back(create_subscription<geometry_msgs::msg::PoseStamped>(
                "/" + id + "/pose", 10,
                [this, id](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                    handlePose(id, msg);
                }, sub_opt));

            // Battery
            battery_subs_.push_back(create_subscription<sensor_msgs::msg::BatteryState>(
                "/" + id + "/battery_status", 10,
                [this, id](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
                    handleBattery(id, msg);
                }, sub_opt));

            // Cmd position (to detect commanded landings)
            cmd_subs_.push_back(create_subscription<crazyflie_interfaces::msg::Position>(
                "/" + id + "/cmd_position", 10,
                [this, id](const crazyflie_interfaces::msg::Position::SharedPtr msg) {
                    handleCmd(id, msg);
                }, sub_opt));

            // Initialize per-drone state
            drone_in_base_[id] = true;   // start at base
            drone_in_env_[id] = true;
            drone_airborne_[id] = false;
            drone_last_z_[id] = 0.0;
            drone_collided_[id] = false;
            drone_prev_vel_[id] = {0, 0, 0};
            drone_prev_pos_[id] = geometry_msgs::msg::Point();
            drone_prev_time_[id] = rclcpp::Time(0, 0, RCL_ROS_TIME);
            drone_has_prev_[id] = false;
        }

        // AGV pose
        agv_sub_ = create_subscription<geometry_msgs::msg::Point>(
            "/AGV/pose", 10,
            [this](const geometry_msgs::msg::Point::SharedPtr msg) {
                handleAGVPose(msg);
            }, sub_opt);

        // RTH state
        rth_sub_ = create_subscription<std_msgs::msg::Bool>(
            "RTH_STATE", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                handleRTH(msg);
            }, sub_opt);

        // Target found (marker detections)
        rclcpp::QoS qos(rclcpp::KeepLast(100));
        qos.reliable();
        qos.transient_local();
        qos.durability_volatile();
        target_sub_ = create_subscription<icuas25_msgs::msg::TargetInfo>(
            "/target_found", qos,
            [this](const icuas25_msgs::msg::TargetInfo::SharedPtr msg) {
                handleTargetFound(msg);
            });

        // Mission drone (all-landed signal)
        mission_drone_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/mission_drone", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data && !mission_ended_) {
                    mission_ended_ = true;
                    mission_end_time_sec_ = missionSeconds();
                    logEvent("Mission completed – all drones returned to base.");
                    printScoreCard();
                }
            }, sub_opt);

        // z_target from BFS
        z_target_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/mission/z_target", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                double old_z = current_z_target_.load();
                double new_z = msg->data;
                current_z_target_.store(new_z);
                if (std::abs(new_z - old_z) > 0.05 && old_z > 0.01) {
                    logEvent("Mission z_target changed: " + fmtd(old_z) + " -> " + fmtd(new_z));
                }
            }, sub_opt);

        // Waypoint array (BFS output)
        waypoint_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
            "/drone/waypoint_array", 10,
            [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
                size_t n = msg->poses.size();
                if (n != last_waypoint_count_) {
                    last_waypoint_count_ = n;
                }
            }, sub_opt);

        octomap_client_ = create_client<octomap_msgs::srv::GetOctomap>("octomap_binary");

        diag_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&DebugLoggerNode::diagnosticsTick, this));

        scorecard_timer_ = create_wall_timer(
            std::chrono::seconds(10),
            std::bind(&DebugLoggerNode::printScoreCard, this));

        mission_start_time_ = now();
        last_eval_time_ = 0.0;
        logEvent("Mission Log Start");

        RCLCPP_INFO(get_logger(),
            "=== DebugLoggerNode online  |  %d drones  |  comm_range=%.0f m ===",
            num_robots_, comm_range_);
    }

    ~DebugLoggerNode() {
        if (log_file_.is_open()) {
            log_file_.close();
        }
    }

private:

    void printScoreCardLocked() {
        double t_mission;
        if (timed_out_) {
            t_mission = mission_end_time_sec_;
        } else if (mission_ended_) {
            t_mission = mission_end_time_sec_;
        } else {
            t_mission = missionSeconds();
        }
        
        if (t_mission <= 0.0) t_mission = 1.0;
        
        // Exact maths from task.md
        double points_conn = 30.0 * (total_connected_time_ / t_mission);
        
        double t_mission_max = 164.5; // Used Eval C max time as reference
        double points_time = 30.0 * (t_mission / t_mission_max);
        
        double err_avg = 0.0;
        if (!marker_errors_.empty()) {
            double sum = 0.0;
            for (auto const& [id, err] : marker_errors_) sum += err;
            err_avg = sum / marker_errors_.size();
        } else {
            err_avg = 10.0;
        }
        
        double err_min = 0.0; 
        double marker_precision = std::exp(1.0 - 2.0 * (err_avg - 10.0 * err_min));
        double marker_precision_max = std::exp(1.0); 
        double marker_precision_rel = marker_precision / marker_precision_max;
        
        int markers_detected = markers_found_.size();
        int markers_total = 6;
        double points_detection = 40.0 * ((double)markers_detected / markers_total) * marker_precision_rel;
        
        double raw_total = points_conn + points_time + points_detection;
        double multiplier = std::pow(0.9, infractions_);
        double final_score = raw_total * multiplier;
        
        std::ostringstream ss;
        ss << "\\n============================================\\n"
           << (mission_ended_ || timed_out_ ? "              FINAL SCORECARD               \\n" : "              LIVE SCORECARD                \\n")
           << "============================================\\n"
           << " Mission Time:    " << fmtd(t_mission) << " s  (Max Ref: " << fmtd(t_mission_max) << " s)\\n"
           << " Connected Time:  " << fmtd(total_connected_time_) << " s\\n"
           << " Targets Found:   " << markers_detected << " / " << markers_total << " (Avg Error: " << fmtd(err_avg, 3) << " m)\\n"
           << " Precision Rel:   " << fmtd(marker_precision_rel, 3) << "\\n"
           << " Infractions:     " << infractions_ << " (Multiplier: " << fmtd(multiplier, 3) << ")\\n"
           << "--------------------------------------------\\n"
           << " Points (Conn):   " << fmtd(points_conn) << " / 30.0\\n"
           << " Points (Time):   " << fmtd(points_time) << " / 30.0\\n"
           << " Points (Detect): " << fmtd(points_detection) << " / 40.0\\n"
           << "--------------------------------------------\\n"
           << " ESTIMATED SCORE: " << fmtd(final_score) << " / 100.0\\n"
           << "============================================\\n";
           
        RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());
        
        if (log_file_.is_open()) {
            std::lock_guard<std::mutex> log_lock(log_mutex_);
            log_file_ << ss.str();
            log_file_.flush();
        }
    }

    void printScoreCard() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        printScoreCardLocked();
    }

    std::string fmtd(double v, int prec = 2) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(prec) << v;
        return ss.str();
    }

    double missionSeconds() {
        return (now() - mission_start_time_).seconds();
    }

    void logEvent(const std::string& msg) {
        std::lock_guard<std::mutex> lock(log_mutex_);
        double t = missionSeconds();
        std::string line = fmtd(t) + ": " + msg;

        RCLCPP_INFO(get_logger(), "%s", line.c_str());

        if (log_file_.is_open()) {
            log_file_ << line << "\n";
            log_file_.flush();
        }

        log_lines_.push_back(line);
    }

    std::string idList(const std::vector<std::string>& ids) {
        std::string s = "[";
        for (size_t i = 0; i < ids.size(); ++i) {
            if (i) s += ", ";
            s += "'" + ids[i] + "'";
        }
        return s + "]";
    }

    void handlePose(const std::string& id,
                    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        double x = msg->pose.position.x;
        double y = msg->pose.position.y;
        double z = msg->pose.position.z;

        // Store latest pose for network-graph computation
        latest_poses_[id] = msg->pose.position;

        // ---- Base proximity (within 0.5 m of origin at ground) ----
        double dist_base = std::hypot(x, y);
        bool near_base = (dist_base < 1.0);

        // Detect leave base
        if (drone_in_base_[id] && !near_base && z > 0.3) {
            drone_in_base_[id] = false;

            // Collect all drones leaving base at roughly the same time
            auto t = now();
            pending_base_leave_ids_.push_back(id);
            pending_base_leave_time_ = t;

            // Defer log by a short period so we can batch
            if (!base_leave_timer_) {
                base_leave_timer_ = create_wall_timer(
                    std::chrono::milliseconds(500),
                    [this]() {
                        std::lock_guard<std::mutex> lk(data_mutex_);
                        if (!pending_base_leave_ids_.empty()) {
                            logEvent("UAVs " + idList(pending_base_leave_ids_) + " left the base.");
                            pending_base_leave_ids_.clear();
                        }
                        base_leave_timer_->cancel();
                        base_leave_timer_ = nullptr;
                    });
            }
        }

        // Detect enter base
        if (!drone_in_base_[id] && near_base && z < 0.3) {
            drone_in_base_[id] = true;

            pending_base_enter_ids_.push_back(id);
            if (!base_enter_timer_) {
                base_enter_timer_ = create_wall_timer(
                    std::chrono::milliseconds(500),
                    [this]() {
                        std::lock_guard<std::mutex> lk(data_mutex_);
                        if (!pending_base_enter_ids_.empty()) {
                            logEvent("UAVs " + idList(pending_base_enter_ids_) + " entered the base.");
                            pending_base_enter_ids_.clear();
                        }
                        base_enter_timer_->cancel();
                        base_enter_timer_ = nullptr;
                    });
            }
        }

        // ---- Environment bounds ----
        bool in_env = (x > env_min_x_ && x < env_max_x_ &&
                       y > env_min_y_ && y < env_max_y_ &&
                       z >= 0.0 && z < env_max_z_);
        if (drone_in_env_[id] && !in_env) {
            drone_in_env_[id] = false;
            logEvent("UAVs ['" + id + "'] has left the environment bounds.");
            infractions_++;
        } else if (!drone_in_env_[id] && in_env) {
            drone_in_env_[id] = true;
            
        }

        // ---- Collision detection via sudden velocity change ----
        if (drone_has_prev_[id]) {
            double dt = (msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9) -
                        (drone_prev_time_[id].seconds());
            if (dt > 0.01 && dt < 1.0) {
                double vx = (x - drone_prev_pos_[id].x) / dt;
                double vy = (y - drone_prev_pos_[id].y) / dt;
                double vz = (z - drone_prev_pos_[id].z) / dt;

                double prev_speed = std::sqrt(
                    drone_prev_vel_[id][0] * drone_prev_vel_[id][0] +
                    drone_prev_vel_[id][1] * drone_prev_vel_[id][1] +
                    drone_prev_vel_[id][2] * drone_prev_vel_[id][2]);
                double cur_speed = std::sqrt(vx*vx + vy*vy + vz*vz);

                // If moving and sudden deceleration > threshold → collision
                double decel = prev_speed - cur_speed;
                // Relaxed the thresholds to prevent false positives from simulation stuttering or sharp manual stops
                if (prev_speed > 1.0 && decel > 3.0 && !drone_collided_[id]) {
                    drone_collided_[id] = true;
                    logEvent("UAVs ['" + id + "'] have collided with an obstacle.");
                    infractions_++;
                }

                // Reset collision flag once moving smoothly again
                if (cur_speed > 0.3 && std::abs(decel) < 0.5) {
                    drone_collided_[id] = false;
                }

                drone_prev_vel_[id] = {vx, vy, vz};
            }
        }

        drone_prev_pos_[id] = msg->pose.position;
        drone_prev_time_[id] = rclcpp::Time(msg->header.stamp);
        drone_has_prev_[id] = true;
        drone_last_z_[id] = z;

        // Airborne tracking
        drone_airborne_[id] = (z > 0.15);
    }

    
    // Battery handler
    void handleBattery(const std::string& id,
                       const sensor_msgs::msg::BatteryState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        float pct = msg->percentage;
        float prev = drone_battery_pct_[id];
        drone_battery_pct_[id] = pct;

        // Log critical battery events
        if (prev >= 25.0f && pct < 25.0f) {
            logEvent("CRITICAL: " + id + " battery below 25% (" + fmtd(pct, 1) + "%)!");
            infractions_++;
        }
        if (prev >= 70.0f && pct < 70.0f) {
            logEvent(id + " battery below charging threshold (" + fmtd(pct, 1) + "%).");
        }
    }

    // Cmd position handler – detect commanded landings on markers
    void handleCmd(const std::string& id,
                   const crazyflie_interfaces::msg::Position::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);

        // Detect when a drone is commanded to land (z ≈ 0)
        if (msg->z < 0.1 && drone_airborne_[id]) {
            // Check if it's landing in the charging area
            if (has_charging_area_) {
                double x = msg->x, y = msg->y;
                double min_x = std::min(charging_ul_[0], charging_dr_[0]);
                double max_x = std::max(charging_ul_[0], charging_dr_[0]);
                double min_y = std::min(charging_ul_[1], charging_dr_[1]);
                double max_y = std::max(charging_ul_[1], charging_dr_[1]);

                bool in_charging = (x >= min_x && x <= max_x && y >= min_y && y <= max_y);
                if (!in_charging) {
                    // Could be on a marker or outside
                    logEvent("UAVs ['" + id + "'] landed outside the charging area and landing pads.");
                }
            }
        }
    }

    // Target found handler
    void handleTargetFound(const icuas25_msgs::msg::TargetInfo::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        int mid = msg->id;

        if (markers_found_.find(mid) == markers_found_.end()) {
            markers_found_.insert(mid);
            logEvent("UAV cf_3 landed on landing platform (marker ID " +
                     std::to_string(mid) + ").");
        }
        
        auto it = GROUND_TRUTH.find(mid);
        if (it != GROUND_TRUTH.end()) {
            double gt_x = std::get<0>(it->second);
            double gt_y = std::get<1>(it->second);
            double gt_z = std::get<2>(it->second);
            double err = std::sqrt(std::pow(msg->location.x - gt_x, 2) + 
                                   std::pow(msg->location.y - gt_y, 2) + 
                                   std::pow(msg->location.z - gt_z, 2));
            marker_errors_[mid] = err;
        }
    }

    // AGV pose handler
    void handleAGVPose(const geometry_msgs::msg::Point::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        agv_pos_ = *msg;
        has_agv_pos_ = true;
    }

    // RTH state handler
    void handleRTH(const std_msgs::msg::Bool::SharedPtr msg) {
        bool was = rth_active_.load();
        rth_active_.store(msg->data);
        if (msg->data && !was) {
            logEvent("Return-To-Home initiated.");
        } else if (!msg->data && was) {
            logEvent("Return-To-Home cancelled / completed.");
        }
    }

    // Network Raycast Helper (skips endpoints to ignore drone bodies/ground)
    bool isLineOfSightClear(const octomap::point3d& p1, const octomap::point3d& p2, double dist) {
        if (!tree_) return true;
        if (dist < 1.0) return true; // too close, assume clear

        double step = tree_->getResolution();
        if (step <= 0.0) step = 0.15;

        octomap::point3d dir = p2 - p1;
        octomap::point3d dir_norm = dir * (1.0 / dist);

        // Skip the first and last 0.5 meters so we don't raycast inside the drones' own boundaries
        for (double d = 0.5; d < dist - 0.5; d += step) {
            octomap::point3d pt = p1 + dir_norm * d;
            
            // Ignore the floor layer
            if (pt.z() < 0.2) continue;

            octomap::OcTreeNode* node = tree_->search(pt);
            if (node && tree_->isNodeOccupied(node)) {
                return false; // occupied space blocks the signal
            }
        }
        return true;
    }

    // Diagnostics timer (1 Hz) – network connectivity checks
    void diagnosticsTick() {
        // Fetch map every 5 ticks to avoid spamming the service
        if (octomap_ticks_++ % 5 == 0 && octomap_client_ && octomap_client_->service_is_ready()) {
            auto req = std::make_shared<octomap_msgs::srv::GetOctomap::Request>();
            octomap_client_->async_send_request(req, [this](rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedFuture future) {
                auto res = future.get();
                octomap::AbstractOcTree* abs_tree = octomap_msgs::binaryMsgToMap(res->map);
                std::lock_guard<std::mutex> lk(data_mutex_);
                tree_.reset(dynamic_cast<octomap::OcTree*>(abs_tree));
                if (tree_) {
                    double x_min, y_min, z_min, x_max, y_max, z_max;
                    tree_->getMetricMin(x_min, y_min, z_min);
                    tree_->getMetricMax(x_max, y_max, z_max);

                    env_min_x_ = std::min(-15.0, x_min);
                    env_max_x_ = std::max(15.0, x_max);
                    env_min_y_ = std::min(-15.0, y_min);
                    env_max_y_ = std::max(15.0, y_max);
                    env_max_z_ = std::max(10.0, z_max);
                }
            });
        }

        std::lock_guard<std::mutex> lock(data_mutex_);

        double now_sec = missionSeconds();
        if (!mission_ended_ && !timed_out_) {
            double dt = now_sec - last_eval_time_;
            if (!network_disconnected_) {
                total_connected_time_ += dt;
            } else {
                if (now_sec - network_disconnect_time_sec_ > 180.0) {
                    timed_out_ = true;
                    mission_end_time_sec_ = network_disconnect_time_sec_;
                    logEvent("Mission timed out due to 3+ minutes of continuous disconnection.");
                    printScoreCardLocked();
                }
            }
            last_eval_time_ = now_sec;
        }

        // ------- Network connectivity graph ------

        if (latest_poses_.size() < drone_ids_.size() || !has_agv_pos_)
            return;

        // Nodes: 0..N-1 = drones, N = AGV, N+1 = Base
        int N = static_cast<int>(drone_ids_.size());
        std::vector<std::vector<int>> adj(N + 2);
        
        geometry_msgs::msg::Point base_pos;
        base_pos.x = 0; base_pos.y = 0; base_pos.z = 0.5; // elevate base slightly avoiding floor

        for (int i = 0; i < N; ++i) {
            auto& pi = latest_poses_[drone_ids_[i]];
            // drone↔Base
            double d_base = std::sqrt(
                std::pow(pi.x - base_pos.x, 2) +
                std::pow(pi.y - base_pos.y, 2) +
                std::pow(pi.z - base_pos.z, 2));
            if (d_base <= comm_range_) {
                if (isLineOfSightClear(octomap::point3d(pi.x, pi.y, pi.z), 
                                       octomap::point3d(base_pos.x, base_pos.y, base_pos.z), 
                                       d_base)) {
                    adj[i].push_back(N + 1);
                    adj[N + 1].push_back(i);
                }
            }
            
            // drone↔AGV
            double d_agv = std::sqrt(
                std::pow(pi.x - agv_pos_.x, 2) +
                std::pow(pi.y - agv_pos_.y, 2) +
                std::pow(pi.z - agv_pos_.z, 2));
            if (d_agv <= comm_range_) {
                if (isLineOfSightClear(octomap::point3d(pi.x, pi.y, pi.z), 
                                       octomap::point3d(agv_pos_.x, agv_pos_.y, agv_pos_.z), 
                                       d_agv)) {
                    adj[i].push_back(N);
                    adj[N].push_back(i);
                }
            }
            // drone↔drone
            for (int j = i + 1; j < N; ++j) {
                auto& pj = latest_poses_[drone_ids_[j]];
                double d = std::sqrt(
                    std::pow(pi.x - pj.x, 2) +
                    std::pow(pi.y - pj.y, 2) +
                    std::pow(pi.z - pj.z, 2));
                if (d <= comm_range_) {
                    if (isLineOfSightClear(octomap::point3d(pi.x, pi.y, pi.z), 
                                           octomap::point3d(pj.x, pj.y, pj.z), 
                                           d)) {
                        adj[i].push_back(j);
                        adj[j].push_back(i);
                    }
                }
            }
        }
        
        // AGV↔Base
        double d_agv_base = std::sqrt(
            std::pow(agv_pos_.x - base_pos.x, 2) +
            std::pow(agv_pos_.y - base_pos.y, 2) +
            std::pow(agv_pos_.z - base_pos.z, 2));
        if (d_agv_base <= comm_range_) {
            if (isLineOfSightClear(octomap::point3d(agv_pos_.x, agv_pos_.y, agv_pos_.z), 
                                   octomap::point3d(base_pos.x, base_pos.y, base_pos.z), 
                                   d_agv_base)) {
                adj[N].push_back(N + 1);
                adj[N + 1].push_back(N);
            }
        }

        // BFS from Base node to find all reachable drones and AGV
        std::vector<bool> reachable(N + 2, false);
        std::queue<int> q;
        q.push(N + 1); // Start at Base
        reachable[N + 1] = true;
        while (!q.empty()) {
            int u = q.front(); q.pop();
            for (int v : adj[u]) {
                if (!reachable[v]) {
                    reachable[v] = true;
                    q.push(v);
                }
            }
        }

        // Determine disconnected drones
        std::vector<std::string> disconnected;
        for (int i = 0; i < N; ++i) {
            if (!reachable[i]) disconnected.push_back(drone_ids_[i]);
        }
        if (!reachable[N]) disconnected.push_back("AGV");

        bool any_disconnected = !disconnected.empty();

        // Checking for connected components (network bifurcation)
        std::vector<bool> visited(N + 2, false);
        int components = 0;
        for (int start = 0; start <= N + 1; ++start) {
            if (visited[start]) continue;
            // Only counting components that have at least one active connection or node
            components++;
            std::queue<int> bfs_q;
            bfs_q.push(start);
            visited[start] = true;
            while (!bfs_q.empty()) {
                int u = bfs_q.front(); bfs_q.pop();
                for (int v : adj[u]) {
                    if (!visited[v]) {
                        visited[v] = true;
                        bfs_q.push(v);
                    }
                }
            }
        }
        bool bifurcated = (components > 1);

        // State transitions

        if (any_disconnected && !network_disconnected_) {
            // Network just disconnected
            network_disconnected_ = true;
            network_disconnect_time_ = now();
            network_disconnect_time_sec_ = missionSeconds();

            // Determining the  reason why
            std::string reason;
            if (bifurcated) {
                reason = "Network bifurcation";
            } else {
                // Specific drone disconnected
                reason = disconnected[0] + " disconnected";
            }
            logEvent("Network has disconnected. Info: " + reason + ".");
        }
        else if (any_disconnected && network_disconnected_) {
            // Still disconnected  check if we should log duration
            double dur = (now() - network_disconnect_time_).seconds();

            // Check if bifurcation status changed
            if (bifurcated && !was_bifurcated_) {
                logEvent("Network has disconnected. Info: Network bifurcation.");
            }

            // Log extended disconnect at 60s intervals
            int dur_bucket = static_cast<int>(dur) / 60;
            if (dur_bucket > last_disconnect_bucket_) {
                last_disconnect_bucket_ = dur_bucket;
                logEvent("Network disconnected for " + std::to_string(dur_bucket * 60) + "s.");
            }
        }
        else if (!any_disconnected && network_disconnected_) {
            // Network just reconnected
            network_disconnected_ = false;
            last_disconnect_bucket_ = 0;
            logEvent("Network has reconnected.");
        }

        was_bifurcated_ = bifurcated;
    }

    // Data members
    int num_robots_;
    double comm_range_;
    std::vector<std::string> drone_ids_;

    bool has_charging_area_ = false;
    std::vector<double> charging_ul_, charging_dr_;

    double env_min_x_, env_min_y_;
    double env_max_x_, env_max_y_, env_max_z_;

    rclcpp::Time mission_start_time_;
    std::atomic<bool> mission_ended_{false};
    std::atomic<double> current_z_target_{1.0};

    std::mutex data_mutex_;
    std::mutex log_mutex_;
    std::ofstream log_file_;
    std::vector<std::string> log_lines_;

    // Per-drone state
    std::map<std::string, geometry_msgs::msg::Point> latest_poses_;
    std::map<std::string, bool> drone_in_base_;
    std::map<std::string, bool> drone_in_env_;
    std::map<std::string, bool> drone_airborne_;
    std::map<std::string, double> drone_last_z_;
    std::map<std::string, bool> drone_collided_;
    std::map<std::string, std::array<double, 3>> drone_prev_vel_;
    std::map<std::string, geometry_msgs::msg::Point> drone_prev_pos_;
    std::map<std::string, rclcpp::Time> drone_prev_time_;
    std::map<std::string, bool> drone_has_prev_;
    std::map<std::string, float> drone_battery_pct_;

    // Batched base events
    std::vector<std::string> pending_base_leave_ids_;
    rclcpp::Time pending_base_leave_time_;
    rclcpp::TimerBase::SharedPtr base_leave_timer_;

    std::vector<std::string> pending_base_enter_ids_;
    rclcpp::TimerBase::SharedPtr base_enter_timer_;

    // AGV
    geometry_msgs::msg::Point agv_pos_;
    bool has_agv_pos_ = false;

    // RTH
    std::atomic<bool> rth_active_{false};

    // Markers
    std::set<int> markers_found_;
    std::map<int, double> marker_errors_;

    // Network
    bool network_disconnected_ = false;
    rclcpp::Time network_disconnect_time_;
    bool was_bifurcated_ = false;
    int last_disconnect_bucket_ = 0;

    // Scoring variables
    double total_connected_time_ = 0.0;
    double last_eval_time_ = 0.0;
    int infractions_ = 0;
    bool timed_out_ = false;
    double mission_end_time_sec_ = -1.0;
    double network_disconnect_time_sec_ = -1.0;
    rclcpp::TimerBase::SharedPtr scorecard_timer_;

    // Waypoints
    size_t last_waypoint_count_ = 0;

    // Subscriptions
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_subs_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr> battery_subs_;
    std::vector<rclcpp::Subscription<crazyflie_interfaces::msg::Position>::SharedPtr> cmd_subs_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr agv_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr rth_sub_;
    rclcpp::Subscription<icuas25_msgs::msg::TargetInfo>::SharedPtr target_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mission_drone_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr z_target_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr waypoint_sub_;
    rclcpp::TimerBase::SharedPtr diag_timer_;

    rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedPtr octomap_client_;
    std::shared_ptr<octomap::OcTree> tree_;
    int octomap_ticks_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DebugLoggerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
