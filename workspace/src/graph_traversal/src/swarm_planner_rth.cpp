#include "swarm_planner.hpp"

void SwarmPlanner::handleReturnToHome() {
    if (landing_service_called_) {
         std::lock_guard<std::mutex> lock(data_mutex_);
         active_commands_.clear();
         return;
    }

    // Sequential Logic: One by one return

    std::shared_ptr<const nav_msgs::msg::OccupancyGrid> local_grid_ptr;
    std::map<std::string, geometry_msgs::msg::PoseStamped> local_poses;
    std::map<std::string, geometry_msgs::msg::Point> home_poses;

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        local_grid_ptr = cached_grid_; // refcount bump, not a copy
        local_poses = swarm_poses_;
        home_poses = initial_poses_;
    }
    if (!local_grid_ptr) return;
    const nav_msgs::msg::OccupancyGrid& local_grid = *local_grid_ptr;

    std::map<std::string, std::vector<crazyflie_interfaces::msg::Position>> new_commands;
    double z_safe = 1.0; // Fly home at safe height

    // 0. Compute RTH Order (ONCE)
    if (!rth_order_computed_) {
         std::lock_guard<std::mutex> lock(data_mutex_);
         rth_sorted_ids_ = droneIds_; // Start with default

         // Sort by Battery Percentage (Ascending: Lowest First)
         std::sort(rth_sorted_ids_.begin(), rth_sorted_ids_.end(),
             [this](const std::string& a, const std::string& b) {
                 return battery_states_[a].percentage < battery_states_[b].percentage;
             });

         rth_order_computed_ = true;

         RCLCPP_INFO(this->get_logger(), "RTH Sequence Computed (Lowest Battery First):");
         for(const auto& id : rth_sorted_ids_) {
             RCLCPP_INFO(this->get_logger(), " - %s (%.1f%%)", id.c_str(), battery_states_[id].percentage);
         }
    }

    // 1. Generate Commands (serial: only NUM_ROBOTS=5 drones, OMP overhead
    // would dominate; also avoids the omp critical blocks we used to need
    // around rth_index_ and new_commands).
    for (int i = 0; i < (int)rth_sorted_ids_.size(); ++i) {
        std::string id = rth_sorted_ids_[i];

        if (!home_poses.count(id)) continue;

        auto current_pos = local_poses[id].pose.position;
        auto home_pos = home_poses[id];

        std::vector<crazyflie_interfaces::msg::Position> cmd_list;

        // --- SEQUENTIAL LOGIC ---
        if (i < rth_index_) {
            // ALREADY LANDED -> STAY DOWN
            drone_roles_[id] = DroneRole::LANDING;
            crazyflie_interfaces::msg::Position cmd;
            cmd.x = home_pos.x; cmd.y = home_pos.y; cmd.z = 0.05; cmd.yaw = 0.0;
            cmd_list.push_back(cmd);
        }
        else if (i > rth_index_) {
             // WAITING -> HOVER IN PLACE (returning to base)
             drone_roles_[id] = DroneRole::RTH;
             crazyflie_interfaces::msg::Position cmd;
             cmd.x = current_pos.x; cmd.y = current_pos.y; cmd.z = current_pos.z; cmd.yaw = 0.0;
             cmd_list.push_back(cmd);
        }
        else {
            // ACTIVE DRONE (i == rth_index_) -> GO HOME
            drone_roles_[id] = DroneRole::RTH;

            // Check distance to home (2D)
            double dist = std::hypot(current_pos.x - home_pos.x, current_pos.y - home_pos.y);

            if (dist < 0.2) {
                // ARRIVED AT HOME -> LAND
                drone_roles_[id] = DroneRole::LANDING;
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
                    // Drone has landed. Move to next.
                    if (i == rth_index_) {
                        rth_index_++;
                        RCLCPP_INFO(this->get_logger(), "Drone %s Landed. Next!", id.c_str());
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

        new_commands[id] = cmd_list;
    }

    // 2. Commit
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        active_commands_ = new_commands;
    }

    // 3. Final Landing Check
    // If all drones have completed sequential return (rth_index_ has passed the last index)
    if (rth_index_ >= (int)droneIds_.size()) {
        if (!landing_service_called_) {
            RCLCPP_INFO(this->get_logger(), "All drones at home position (Z=0.05). Triggering Final Land Service for ALL.");

            // Call Land Service for everyone
            for (const auto& id : droneIds_) {
                auto client = land_clients_[id];
                if (client->service_is_ready()) {
                    auto req = std::make_shared<crazyflie_interfaces::srv::Land::Request>();
                    req->height = 0.0; // Land on ground
                    req->duration.sec = 2; // Duration for landing
                    client->async_send_request(req);
                    RCLCPP_INFO(this->get_logger(), "🛬 Called Land Service for %s", id.c_str());
                }
            }
            landing_service_called_ = true;
        }

        // Publish mission completion once when all drones are back at base
        if (!mission_drone_published_) {
            std_msgs::msg::Bool mission_msg;
            mission_msg.data = true;
            mission_drone_pub_->publish(mission_msg);
            mission_drone_published_ = true;
            RCLCPP_INFO(this->get_logger(), "Published /mission_drone = true (all drones returned to base).");
        }
    }
}
