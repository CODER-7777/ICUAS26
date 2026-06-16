#include "swarm_planner.hpp"

// Iterative target-separation enforcement. If two targets at similar
// altitude sit closer than min_sep in XY, push them apart along the
// connecting line until they're exactly min_sep apart. Iterating a few
// times absorbs chains where moving A away from B brings A close to C.
void SwarmPlanner::enforceTargetSeparation(geometry_msgs::msg::PoseArray& targets,
                                           double min_sep, double z_thresh) {
    if (min_sep <= 0.0) return;
    const int max_iters = 6;
    for (int iter = 0; iter < max_iters; ++iter) {
        bool moved = false;
        for (size_t i = 0; i + 1 < targets.poses.size(); ++i) {
            for (size_t j = i + 1; j < targets.poses.size(); ++j) {
                auto& a = targets.poses[i].position;
                auto& b = targets.poses[j].position;
                if (std::abs(a.z - b.z) > z_thresh) continue;
                double dx = b.x - a.x;
                double dy = b.y - a.y;
                double d = std::hypot(dx, dy);
                if (d >= min_sep) continue;
                double ux, uy;
                if (d < 1e-6) {
                    ux = 1.0; uy = 0.0;
                } else {
                    ux = dx / d; uy = dy / d;
                }
                double need = (min_sep - d) * 0.5;
                a.x -= ux * need; a.y -= uy * need;
                b.x += ux * need; b.y += uy * need;
                moved = true;
            }
        }
        if (!moved) break;
    }
}

// Drone planning priority (used by prioritized cooperative A*).
// Higher = plans first; lower-priority drones must route around its path.
//   30xx: charging-related (constrained vertical motion, plan first)
//   20xx: actively assigned to a mission target
//   10xx: idle / shadow position
// Within each band, lower drone-id (cf_1) gets higher priority — gives
// deterministic, tie-broken ordering.
int SwarmPlanner::planPriority(const std::string& id, bool is_assigned) const {
    int base;
    auto it = battery_states_.find(id);
    if (it != battery_states_.end()) {
        const auto& bs = it->second;
        if (bs.is_charging || bs.is_going_to_charge || bs.is_leaving_charger) {
            base = 30;
        } else if (is_assigned) {
            base = 20;
        } else {
            base = 10;
        }
    } else {
        base = is_assigned ? 20 : 10;
    }
    int magic = (id.size() > 3 && std::isdigit(static_cast<unsigned char>(id[3])))
                    ? (id[3] - '0') : 9;
    return base * 100 - magic;
}

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

    publishRoles();
}

const char* SwarmPlanner::roleToString(DroneRole r) {
    switch (r) {
        case DroneRole::RTH:             return "RTH";
        case DroneRole::LANDING:         return "LANDING";
        case DroneRole::CHAIN_COMPONENT: return "CHAIN_COMPONENT";
        case DroneRole::SEARCH:          return "SEARCH";
        default:                         return "UNKNOWN";
    }
}

void SwarmPlanner::publishRoles() {
    std::string msg;
    for (size_t i = 0; i < droneIds_.size(); ++i) {
        if (i > 0) msg += ",";
        msg += droneIds_[i] + ":" + roleToString(drone_roles_[droneIds_[i]]);
    }
    std_msgs::msg::String out;
    out.data = msg;
    drone_role_pub_->publish(out);
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

    // Pre-matching: nudge mission targets apart so two drones are never
    // assigned to spots that overlap at the controller. Mutates the local
    // copy only; upstream waypoint topic is untouched.
    enforceTargetSeparation(
        local_targets,
        this->get_parameter("target_min_separation").as_double(),
        this->get_parameter("reservation_z_threshold").as_double());

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

    // --- INITIALIZE ROLES: default SEARCH for all drones ---
    {
        for (const auto& id : droneIds_) {
            drone_roles_[id] = DroneRole::SEARCH;
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


    // 4a. Build list of "Connection Anchors" (Base + Active Drone Positions)
    // Used by fallback shadow logic to place idle drones near real drone
    // positions rather than abstract waypoint targets.
    std::vector<geometry_msgs::msg::Point> anchors;

    geometry_msgs::msg::Point base;
    base.x = 0.0; base.y = 0.0; base.z = z_mission;
    anchors.push_back(base);

    // Mirror the planner's base anchor to swarm_viz. Republished every tick
    // because z tracks z_mission, which AGV-loop height boosts can change.
    if (base_anchor_pub_) {
        geometry_msgs::msg::PointStamped bs;
        bs.header.frame_id = "world";
        bs.header.stamp = this->now();
        bs.point = base;
        base_anchor_pub_->publish(bs);
    }

    for (const auto& kv : drone_to_target) {
        const std::string& aid = available_drones[kv.first];
        auto pit = local_poses.find(aid);
        if (pit != local_poses.end()) {
            anchors.push_back(pit->second.pose.position);
        }
    }

    // Prioritized cooperative zone selection for idle drones.
    //   - Reset zone locks at start of tick so visited+pillar+lock state is
    //     evaluated fresh by SearchManager.
    //   - Collect every drone's CURRENT position as obstacles up-front
    //     (chain drones + idle drones). Idle drones planned earlier in
    //     this loop also append their TARGET zone position so later idle
    //     drones treat it as occupied.
    //   - Idle drones are processed in priority order (cf_1 first, then
    //     cf_2, ...) so deterministic tie-breaking matches A* phase.
    search_mgr_.resetLocks();

    std::vector<geometry_msgs::msg::Point> drone_obstacles;
    drone_obstacles.reserve(num_drones);
    for (const auto& kv : local_poses) {
        drone_obstacles.push_back(kv.second.pose.position);
    }

    // Sort available_drones into priority order for idle picks. Active
    // (chain) drones get processed in the same loop but their job is
    // determined by drone_to_target — only the idle pick logic uses the
    // priority order to claim zones first-come-first-served.
    std::vector<size_t> avail_order(num_avail);
    for (size_t k = 0; k < num_avail; ++k) avail_order[k] = k;
    std::sort(avail_order.begin(), avail_order.end(),
        [&](size_t a, size_t b) {
            bool a_assigned = drone_to_target.count(a) > 0;
            bool b_assigned = drone_to_target.count(b) > 0;
            return planPriority(available_drones[a], a_assigned)
                 > planPriority(available_drones[b], b_assigned);
        });

    // A. Process AVAILABLE DRONES (in priority order for cooperative locking)
    for (size_t order_k = 0; order_k < num_avail; ++order_k) {
        size_t i = avail_order[order_k];
        // Find which global index this drone is
        std::string id = available_drones[i];

        // Find global index for "jobs" array
        auto it = std::find(droneIds_.begin(), droneIds_.end(), id);
        size_t global_idx = std::distance(droneIds_.begin(), it);

        jobs[global_idx].id = id;
        auto current_pos = local_poses[id].pose.position;

        if (drone_to_target.count(i)) {
            // --- ACTIVE DRONE (Chain Component) ---
            drone_roles_[id] = DroneRole::CHAIN_COMPONENT;
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

            // Build obstacle list = every OTHER drone's current position
            // plus zones already locked by earlier-priority idle picks.
            // (Locked zone positions are handled inside SearchManager via
            // locked_zones_; passing current positions catches drones in
            // flight that haven't reached their target yet.)
            std::vector<geometry_msgs::msg::Point> others;
            others.reserve(drone_obstacles.size());
            for (const auto& o : drone_obstacles) {
                if (std::hypot(o.x - current_pos.x, o.y - current_pos.y) < 1e-3
                    && std::abs(o.z - current_pos.z) < 1e-3) continue; // skip self
                others.push_back(o);
            }

            int zone_id = -1;
            if (search_built_ && !search_mgr_.zones_.empty()) {
                zone_id = search_mgr_.pickNextZone(
                    current_pos.x, current_pos.y, current_pos.z,
                    search_anchors, get_comm_range(), losCb, others);
            }

            if (zone_id >= 0) {
                const auto& zn = search_mgr_.zones_[zone_id];
                jobs[global_idx].target_pos.x = zn.x;
                jobs[global_idx].target_pos.y = zn.y;
                jobs[global_idx].target_pos.z = zn.z;
                jobs[global_idx].target_z = zn.z;
                jobs[global_idx].has_search_yaw = true;
                jobs[global_idx].search_yaw = zn.yaw;
                // Lock this zone so subsequent idle drones in this tick
                // cannot claim the same target.
                search_mgr_.lockZone(zone_id);
                // Add the target zone position to drone_obstacles so it
                // also blocks proximity-based picks (drone_clearance_).
                geometry_msgs::msg::Point zp; zp.x = zn.x; zp.y = zn.y; zp.z = zn.z;
                drone_obstacles.push_back(zp);
            } else {
                // Fallback shadow logic (no reachable unvisited zone). Old
                // version used `magic % 4` which collides (e.g. cf_1 and
                // cf_5 both pick -x), pinning two idle drones to the same
                // XY. New version: 8 slots around each anchor, hashed by
                // (anchor_idx + magic) so distinct drones sharing an
                // anchor get distinct slots in steady state.
                int best_anchor_idx = 0;
                double min_dist = 1e9;
                for (size_t k = 0; k < anchors.size(); ++k) {
                    double d = dist3D(current_pos, anchors[k]);
                    if (d < min_dist) { min_dist = d; best_anchor_idx = static_cast<int>(k); }
                }
                geometry_msgs::msg::Point shadow_target = anchors[best_anchor_idx];
                const double offset_dist = 2.0;
                int magic = (id.size() > 3 && std::isdigit(static_cast<unsigned char>(id[3])))
                              ? (id[3] - '0') : 0;
                int slot = ((magic - 1) + best_anchor_idx) & 0x7;
                const double angle = (slot * M_PI) / 4.0;
                shadow_target.x += offset_dist * std::cos(angle);
                shadow_target.y += offset_dist * std::sin(angle);
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

        // Assign RTH role while returning to/from charging area
        drone_roles_[id] = DroneRole::RTH;

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
             drone_roles_[id] = DroneRole::LANDING;
             // HOLD POSITION ON SLOT AT LOW Z
             jobs[global_idx].target_z = 0.05;
             jobs[global_idx].has_search_yaw = true;
             jobs[global_idx].search_yaw = 0.0;
        } else {
             // Going to charge (Transit) -> APPROACH HEIGHT 0.5
             // Safe but distinct from ground
             jobs[global_idx].target_z = 0.5;
             jobs[global_idx].has_search_yaw = true;
             jobs[global_idx].search_yaw = 0.0;
        }
    }

    // C. Process WAITING DRONES - REMOVED PER FEEDBACK
    // (Drones in queue are now in Available list)

    std::vector<std::pair<std::string, std::vector<crazyflie_interfaces::msg::Position>>> new_commands(droneIds_.size());

    // --- Prioritized cooperative planning ---
    // Drones plan one at a time in priority order. Each drone's smoothed
    // path is inflated into a per-altitude reservation grid; subsequent
    // drones treat it as an obstacle. Deterministic, no reactive nudging
    // at the controller.
    std::vector<size_t> plan_order(num_drones);
    for (size_t k = 0; k < num_drones; ++k) plan_order[k] = k;
    std::sort(plan_order.begin(), plan_order.end(), [&](size_t a, size_t b) {
        return planPriority(droneIds_[a], jobs[a].is_assigned)
             > planPriority(droneIds_[b], jobs[b].is_assigned);
    });

    const double reservation_radius = this->get_parameter("reservation_radius").as_double();
    const double z_thresh           = this->get_parameter("reservation_z_threshold").as_double();
    const int reservation_steps     = std::max(0, static_cast<int>(std::ceil(reservation_radius / local_grid.info.resolution)));

    // Per-drone footprint, indexed by drone idx (matches `jobs`). Seeded
    // with the drone's current position so even the first drone to plan
    // sees every other drone as an obstacle. Once a drone plans, its
    // entry is replaced with the smoothed path — strictly more accurate
    // than the current-position seed.
    std::vector<std::vector<geometry_msgs::msg::Point>> footprints(num_drones);
    std::vector<double> footprint_z(num_drones);
    for (size_t k = 0; k < num_drones; ++k) {
        geometry_msgs::msg::Point p = local_poses[droneIds_[k]].pose.position;
        footprints[k] = {p};
        footprint_z[k] = jobs[k].target_z;
    }

    for (size_t order_idx = 0; order_idx < plan_order.size(); ++order_idx) {
        size_t i = plan_order[order_idx];
        DroneJob& job = jobs[i];
        auto current_pos = local_poses[job.id].pose.position;
        std::vector<crazyflie_interfaces::msg::Position> drone_path_cmds;
        std::vector<geometry_msgs::msg::Point> world_path;

        double dist_to_target = std::hypot(current_pos.x - job.target_pos.x, current_pos.y - job.target_pos.y);

        // "Lazy" check: unassigned drone already at its shadow/search spot — hold.
        if (!job.is_assigned && dist_to_target < 0.2) {
            crazyflie_interfaces::msg::Position cmd;
            cmd.x = current_pos.x; cmd.y = current_pos.y; cmd.z = job.target_z;
            cmd.yaw = job.has_search_yaw ? job.search_yaw : 0.0;
            drone_path_cmds.push_back(cmd);
            geometry_msgs::msg::Point p; p.x = current_pos.x; p.y = current_pos.y; p.z = job.target_z;
            world_path.push_back(p);
        } else {
            // Build this drone's planning grid: base grid + every OTHER
            // drone's footprint (within altitude band). Skipping self is
            // essential — otherwise A* can't expand out of its start cell.
            nav_msgs::msg::OccupancyGrid plan_grid = local_grid;
            for (size_t j = 0; j < num_drones; ++j) {
                if (j == i) continue;
                markPathReservation(plan_grid, footprints[j], reservation_steps);
            }

            auto path = runPlanningPipeline(plan_grid, job.target_pos, current_pos, job.target_z);

            if (!path.empty()) {
                world_path = path;
                for (size_t k = 0; k < path.size(); ++k) {
                    crazyflie_interfaces::msg::Position cmd;
                    cmd.x = path[k].x; cmd.y = path[k].y; cmd.z = job.target_z;

                    double dx, dy;
                    if (k == 0) { dx = path[k].x - current_pos.x; dy = path[k].y - current_pos.y; }
                    else        { dx = path[k].x - path[k-1].x;   dy = path[k].y - path[k-1].y; }

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
                // No path: hover and keep our seed-position footprint so
                // others continue avoiding us.
                crazyflie_interfaces::msg::Position cmd;
                cmd.x = current_pos.x; cmd.y = current_pos.y; cmd.z = job.target_z; cmd.yaw = 0.0;
                drone_path_cmds.push_back(cmd);
                geometry_msgs::msg::Point p; p.x = current_pos.x; p.y = current_pos.y; p.z = job.target_z;
                world_path.push_back(p);
            }
        }

        new_commands[i] = {job.id, drone_path_cmds};
        footprints[i] = std::move(world_path);
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
