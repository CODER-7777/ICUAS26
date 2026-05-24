#include "swarm_planner.hpp"

void SwarmPlanner::publishCommands() {
    struct DroneCmd {
        std::string id;
        std::vector<crazyflie_interfaces::msg::Position> waypoints;
    };
    std::vector<DroneCmd> batch_buffer;
    std::map<std::string, geometry_msgs::msg::Point> current_positions;

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (current_state_ != SwarmState::MISSION && current_state_ != SwarmState::RETURN_TO_HOME) return;

        batch_buffer.reserve(active_commands_.size());
        for (const auto& kv : active_commands_) {
            if (!kv.second.empty()) {
                batch_buffer.push_back({kv.first, kv.second});
            }
        }

        for (const auto& kv : swarm_poses_) {
            current_positions[kv.first] = kv.second.pose.position;
        }
    }

    const double max_speed = this->get_parameter("max_speed").as_double();
    constexpr double dt    = 0.05; // 50 ms control loop
    const double max_step  = max_speed * dt;

    for (const auto& item : batch_buffer) {
        auto pub_it = cmd_pubs_.find(item.id);
        if (pub_it == cmd_pubs_.end() || item.waypoints.empty()) continue;
        auto cur_it = current_positions.find(item.id);
        if (cur_it == current_positions.end()) continue;

        size_t target_idx = (item.waypoints.size() > 1) ? 1 : 0;
        crazyflie_interfaces::msg::Position safe_cmd = item.waypoints[target_idx];

        // Speed clamp: interpolate toward the waypoint so the per-tick
        // step never exceeds max_speed * dt. Real flight controllers
        // track gentle setpoint motion much better than teleports.
        const auto& current = cur_it->second;
        double dx = safe_cmd.x - current.x;
        double dy = safe_cmd.y - current.y;
        double dz = safe_cmd.z - current.z;
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (dist > max_step && dist > 1e-6) {
            double scale = max_step / dist;
            safe_cmd.x = current.x + dx * scale;
            safe_cmd.y = current.y + dy * scale;
            safe_cmd.z = current.z + dz * scale;
        }

        pub_it->second->publish(safe_cmd);
    }
}
