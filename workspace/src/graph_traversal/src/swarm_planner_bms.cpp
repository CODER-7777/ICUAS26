#include "swarm_planner.hpp"

void SwarmPlanner::handleBatteryLogic(const std::string& id) {
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

            RCLCPP_INFO(this->get_logger(), " Drone %s CHARGED. Low Takeoff (Z=0.5) & Clearing Area.", id.c_str());
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
            RCLCPP_INFO(this->get_logger(), "Drone %s arrived at charging slot %d. 'Landing' to Z=0.05.", id.c_str(), state.assigned_slot);

            // NO LAND SERVICE - Just state switch + Z command
            state.is_charging = true;
            state.is_going_to_charge = false;
        }
    }
}
