#pragma once

#include "types.hpp"
#include "search_manager.hpp"
#include "utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <octomap/octomap.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <crazyflie_interfaces/msg/position.hpp>
#include <crazyflie_interfaces/srv/takeoff.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <crazyflie_interfaces/srv/land.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <atomic>
#include <mutex>
#include <queue>
#include <map>
#include <unordered_set>
#include <functional>
#include <omp.h>

class SwarmPlanner : public rclcpp::Node {
public:
    SwarmPlanner();

private:
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::TimerBase::SharedPtr timer_, init_timer_, control_timer_;
    rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedPtr octomap_client_;

    SwarmState current_state_;
    bool takeoff_called_ = false;
    const double REACH_THRESHOLD = 0.15;

    std::mutex data_mutex_;
    std::unique_ptr<octomap::OcTree> tree_;
    std::shared_ptr<const nav_msgs::msg::OccupancyGrid> cached_grid_;
    geometry_msgs::msg::PoseArray current_bfsPoints_;
    bool hasPoints_ = false;
    bool map_ready_ = false;

    // Height boost after AGV loop completion
    const double HEIGHT_BOOST = 1.0;
    const double LOOP_DETECTION_RADIUS = 1.0;
    int agv_loop_count_ = 0;
    geometry_msgs::msg::Point agv_start_pos_;
    geometry_msgs::msg::Point agv_current_pos_;
    bool agv_start_captured_ = false;
    bool agv_away_from_start_ = false;
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
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rth_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_drone_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr base_anchor_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_slice_pub_;
    bool rth_state_initialised_ = false;
    bool rth_last_published_ = false;
    std::map<std::string, geometry_msgs::msg::Point> initial_poses_;
    double pole_z_bottom_ = 0.0;
    int rth_index_ = 0;
    bool landing_service_called_ = false;
    bool mission_drone_published_ = false;

    // RTH Sorting Members
    std::vector<std::string> rth_sorted_ids_;
    bool rth_order_computed_ = false;

    // --- BMS Members ---
    std::map<std::string, DroneBatteryState> battery_states_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr> battery_subs_;
    std::map<std::string, rclcpp::Client<crazyflie_interfaces::srv::Land>::SharedPtr> land_clients_;
    std::map<std::string, rclcpp::Client<crazyflie_interfaces::srv::Takeoff>::SharedPtr> takeoff_clients_;

    geometry_msgs::msg::Point charging_station_coords_;
    const float CHARGING_THRESHOLD = 70.0f;
    const float CHARGED_THRESHOLD = 88.0f;

    // --- Charging Slots & Concurrency ---
    std::vector<geometry_msgs::msg::Point> charging_slots_;
    std::vector<bool> slot_occupied_;
    const int MAX_CHARGING_DRONES = 2;

    // --- ArUco Search (per search.md) ---
    SearchManager search_mgr_;
    bool search_built_ = false;

    // --- Drone Role Tracking ---
    std::map<std::string, DroneRole> drone_roles_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr drone_role_pub_;

    // --- Init ---
    void initializeBMS();

    // --- DISTANCE HELPERS ---
    double dist2D(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
        return std::hypot(a.x - b.x, a.y - b.y);
    }

    double dist3D(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        double dz = a.z - b.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    // --- OCTOMAP & GRID UTILS ---
    void fetchOctomapOnce();
    nav_msgs::msg::OccupancyGrid generateOccupancyGrid();

    // --- PLANNING HELPERS ---
    bool hasGridLineOfSight(const idx& start, const idx& end, const nav_msgs::msg::OccupancyGrid& map);
    std::vector<idx> bfs(idx start, idx end, const nav_msgs::msg::OccupancyGrid& map);
    std::vector<geometry_msgs::msg::Point> runPlanningPipeline(
        const nav_msgs::msg::OccupancyGrid& map,
        geometry_msgs::msg::Point target,
        geometry_msgs::msg::Point current_pos,
        double target_z);
    void markPathReservation(nav_msgs::msg::OccupancyGrid& grid,
                             const std::vector<geometry_msgs::msg::Point>& path,
                             int inflation_steps);

    // --- MAIN LOGIC ---
    bool canMatch(int u, double threshold, const std::vector<std::vector<double>>& costs,
                  std::vector<int>& match, std::vector<bool>& vis);
    void runSwarmSystem();
    void handleBatteryLogic(const std::string& id);
    void handleTakeoff();
    void handleMission();
    void handleReturnToHome();
    void enforceTargetSeparation(geometry_msgs::msg::PoseArray& targets,
                                 double min_sep, double z_thresh);
    int planPriority(const std::string& id, bool is_assigned) const;

    // --- ROLE HELPERS ---
    static const char* roleToString(DroneRole r);
    void publishRoles();

    // --- CONTROL ---
    void publishCommands();
};
