#include "swarm_planner.hpp"

geometry_msgs::msg::Point SwarmPlanner::applyRepulsion(
    geometry_msgs::msg::Point target,
    geometry_msgs::msg::Point current,
    const std::map<std::string, geometry_msgs::msg::Point>& all_positions,
    std::string my_id)
{
    // TUNING PARAMETERS
    // Dynamic Safe Radius: Standard 0.7m, but reduced during RTH to allow landing at close start points
    double safe_radius = (current_state_ == SwarmState::RETURN_TO_HOME) ? 0.0 : 0.7;

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

void SwarmPlanner::publishCommands() {
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

    // OMP removed: with batch_buffer.size() == NUM_ROBOTS (5) and a body
    // that's a handful of arithmetic ops + one publish, the fork/join +
    // thread-pool overhead consistently exceeds the work itself.
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

            // 3. SPEED LIMIT: interpolate toward target so step <= max_speed * dt
            double max_speed = this->get_parameter("max_speed").as_double();
            const double dt = 0.05;  // 50ms control loop
            const auto& current = current_positions[item.id];
            double dx = adjusted_pt.x - current.x;
            double dy = adjusted_pt.y - current.y;
            double dz = adjusted_pt.z - current.z;
            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            double max_step = max_speed * dt;
            if (dist > max_step && dist > 1e-6) {
                double scale = max_step / dist;
                adjusted_pt.x = current.x + dx * scale;
                adjusted_pt.y = current.y + dy * scale;
                adjusted_pt.z = current.z + dz * scale;
            }

            // 4. Publish the Adjusted Command
            crazyflie_interfaces::msg::Position safe_cmd = next_wp; // Copy yaw/z
            safe_cmd.x = adjusted_pt.x;
            safe_cmd.y = adjusted_pt.y;
            safe_cmd.z = adjusted_pt.z; // Use clamped/repulsed Z

            pub_it->second->publish(safe_cmd);
        }
    }
}
