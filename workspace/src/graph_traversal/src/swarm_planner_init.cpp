#include "swarm_planner.hpp"

using namespace std::chrono_literals;

SwarmPlanner::SwarmPlanner() : Node("Swarm_planner") {
    this->declare_parameter<double>("inflation_radius", 0.3);
    this->declare_parameter<double>("z_target", 1.0);
    this->declare_parameter<double>("max_speed", 12.0);

    // Drone-drone separation tunables. Collision avoidance now happens at
    // plan time via prioritized cooperative A*, not via reactive repulsion.
    //   target_min_separation: pre-matching nudge so assigned waypoints are
    //     never closer than this in XY (same altitude band).
    //   reservation_radius: inflation around previously-planned drones'
    //     paths when running prioritized A* — the per-drone "corridor" width.
    //   reservation_z_threshold: only treat another drone's path as a
    //     reservation when its z is within this band of ours.
    this->declare_parameter<double>("target_min_separation", 1.0);
    this->declare_parameter<double>("reservation_radius", 0.6);
    this->declare_parameter<double>("reservation_z_threshold", 0.4);
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
                    // Use pole bottom (octomap minZ) as ground reference; fall back to
                    // raw pose z until the map arrives (patched in fetchOctomapOnce).
                    this->initial_poses_[id].z = this->map_ready_ ? this->pole_z_bottom_ : msg->pose.position.z;
                    RCLCPP_INFO(this->get_logger(), "Captured Home Position for %s: (%.2f, %.2f, %.2f)",
                        id.c_str(), msg->pose.position.x, msg->pose.position.y, this->initial_poses_[id].z);
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

    // RTH state publisher (true when Return-To-Home active). transient_local
    // so the ArUco node still sees the latest value even though we only
    // publish on transitions now.
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        rth_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("RTH_STATE", qos);

        // Mission completion publisher: true when all drones have returned to base.
        mission_drone_pub_ = this->create_publisher<std_msgs::msg::Bool>("/mission_done", qos);
    }

    // Drone role publisher: comma-separated "id:ROLE" pairs, updated every planning tick.
    drone_role_pub_ = this->create_publisher<std_msgs::msg::String>("/drone_roles", 10);


    // 4. Timers
    // Init: Get map once. Cancels itself after the map arrives instead of
    // looping forever at 2 Hz.
    init_timer_ = this->create_wall_timer(500ms, [this]() {
        if (map_ready_) {
            if (init_timer_) init_timer_->cancel();
            return;
        }
        fetchOctomapOnce();
    });

    // Planning loop @ 4 Hz (was 10 Hz). Each tick runs 5 A* plans +
    // bipartite matching, so cutting it to 250 ms reclaims significant
    // CPU without hurting tracking quality.
    timer_ = this->create_wall_timer(250ms, std::bind(&SwarmPlanner::runSwarmSystem, this), callback_group_);

    // Control loop stays at 20 Hz — it just re-publishes from
    // active_commands_ with a speed clamp, which is cheap.
    control_timer_ = this->create_wall_timer(50ms, std::bind(&SwarmPlanner::publishCommands, this), callback_group_);

    initializeBMS();
}

void SwarmPlanner::initializeBMS() {
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
