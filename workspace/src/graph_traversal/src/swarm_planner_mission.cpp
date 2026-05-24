#include "swarm_planner.hpp"

bool SwarmPlanner::canMatch(int u, double threshold, const std::vector<std::vector<double>>& costs,
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

void SwarmPlanner::runSwarmSystem() {
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

    // Publish RTH state only on transitions. Sending the same Bool 10x/s
    // wakes the ArUco node's executor for nothing. The publisher uses
    // `transient_local` (configured below) so late subscribers still see
    // the latest value.
    bool rth_now = (current_state_ == SwarmState::RETURN_TO_HOME);
    if (!rth_state_initialised_ || rth_last_published_ != rth_now) {
        std_msgs::msg::Bool rth_msg;
        rth_msg.data = rth_now;
        rth_state_pub_->publish(rth_msg);
        rth_last_published_ = rth_now;
        rth_state_initialised_ = true;
    }
}

void SwarmPlanner::handleTakeoff() {
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

void SwarmPlanner::handleMission() {
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
                    "AGV Loop %d completed! Raising altitude to %.1f m",
                    agv_loop_count_, current_z + HEIGHT_BOOST);
            }
        }
    }


    std::shared_ptr<const nav_msgs::msg::OccupancyGrid> local_grid_ptr;
    geometry_msgs::msg::PoseArray local_targets;
    std::map<std::string, geometry_msgs::msg::PoseStamped> local_poses;

    // CRITICAL: This snapshot ensures we always work on the LATEST points
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        local_grid_ptr = cached_grid_; // refcount bump, not a copy
        local_targets = current_bfsPoints_; // If new points came in, we get them here
        local_poses = swarm_poses_;
    }
    if (!local_grid_ptr) return; // shouldn't happen since map_ready_ gate above
    const nav_msgs::msg::OccupancyGrid& local_grid = *local_grid_ptr;

    // --- SEARCH: mark zones visited by any drone this tick ---
    if (search_built_) {
        for (const auto& kv : local_poses) {
            const auto& p = kv.second.pose.position;
            search_mgr_.markVisited(p.x, p.y, p.z);
        }
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
                         RCLCPP_WARN(this->get_logger(), " Drone %s Low Battery (%.1f%%). Sending to Charge Slot %d.",
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
                RCLCPP_WARN(this->get_logger(), "RECALLING Drone %s (%.1f%%) from charging for MISSION!",
                    recall_id.c_str(), state.percentage);
                // No Service call needed, just state change.
                // Loop will pick up Z=Mission Height next cycle.
             } else {
                RCLCPP_INFO(this->get_logger(), "Redirecting Drone %s (%.1f%%) from charging approach to MISSION.",
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
                 RCLCPP_ERROR(this->get_logger(), "EMERGENCY RTH TRIGGERED! Drone %s Battery at %.1f%% (< 25%%)",
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
        bool has_search_yaw = false;  // override yaw at final waypoint (face pole)
        double search_yaw = 0.0;
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
            // --- IDLE DRONE: ArUco SEARCH per search.md ---
            jobs[global_idx].is_assigned = false;

            // Anchor set for LOS = base station + every active drone's
            // current position (idle drones can chain off active ones).
            std::vector<geometry_msgs::msg::Point> search_anchors;
            geometry_msgs::msg::Point base_pt;
            base_pt.x = 0.0; base_pt.y = 0.0; base_pt.z = z_mission;
            search_anchors.push_back(base_pt);
            for (const auto& kv : drone_to_target) {
                // kv.first = index into available_drones (active set)
                const std::string& aid = available_drones[kv.first];
                auto pit = local_poses.find(aid);
                if (pit != local_poses.end()) {
                    search_anchors.push_back(pit->second.pose.position);
                }
            }

            // LOS predicate uses the inflated 2D grid (Bresenham).
            auto losCb = [&](double ax, double ay, double bx, double by) {
                double res = local_grid.info.resolution;
                double ox = local_grid.info.origin.position.x;
                double oy = local_grid.info.origin.position.y;
                idx s{(int)std::floor((ax - ox) / res), (int)std::floor((ay - oy) / res)};
                idx e{(int)std::floor((bx - ox) / res), (int)std::floor((by - oy) / res)};
                return hasGridLineOfSight(s, e, local_grid);
            };

            int zone_id = -1;
            if (search_built_ && !search_mgr_.zones_.empty()) {
                zone_id = search_mgr_.pickNextZone(
                    current_pos.x, current_pos.y, current_pos.z,
                    search_anchors, get_comm_range(), losCb);
            }

            if (zone_id >= 0) {
                const auto& zn = search_mgr_.zones_[zone_id];
                jobs[global_idx].target_pos.x = zn.x;
                jobs[global_idx].target_pos.y = zn.y;
                jobs[global_idx].target_pos.z = zn.z;
                jobs[global_idx].target_z = zn.z;
                jobs[global_idx].has_search_yaw = true;
                jobs[global_idx].search_yaw = zn.yaw;
            } else {
                // Fallback: legacy shadow logic (no reachable unvisited zone).
                int best_anchor_idx = 0;
                double min_dist = 1e9;
                for (size_t k = 0; k < anchors.size(); ++k) {
                    double d = dist3D(current_pos, anchors[k]);
                    if (d < min_dist) { min_dist = d; best_anchor_idx = k; }
                }
                geometry_msgs::msg::Point shadow_target = anchors[best_anchor_idx];
                double offset_dist = 2.0;
                int magic = id[3] - '0';
                if (magic % 4 == 0)      shadow_target.x += offset_dist;
                else if (magic % 4 == 1) shadow_target.x -= offset_dist;
                else if (magic % 4 == 2) shadow_target.y += offset_dist;
                else                     shadow_target.y -= offset_dist;
                jobs[global_idx].target_pos = shadow_target;
                jobs[global_idx].target_z = z_mission;
            }
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
                         RCLCPP_INFO(this->get_logger(), " Slot %d freed by %s (Cleared dist=%.2fm)", s, id.c_str(), dist_from_center);
                     }
                     battery_states_[id].assigned_slot = -1;
                 }
                 // Drone will become available next cycle
                 RCLCPP_INFO(this->get_logger(), "Drone %s Cleared Charging Area.", id.c_str());

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
             cmd.x = current_pos.x; cmd.y = current_pos.y; cmd.z = job.target_z;
             // Face pole if search zone, otherwise hold yaw 0.
             cmd.yaw = job.has_search_yaw ? job.search_yaw : 0.0;
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
                // search.md step 5: face the pole when entering the
                // viewing zone. Override yaw on the final waypoint.
                if (job.has_search_yaw && !drone_path_cmds.empty()) {
                    drone_path_cmds.back().yaw = job.search_yaw;
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
