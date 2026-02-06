#include <cmath>
#include <cstdlib>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <sstream>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class ConnectivityChecker : public rclcpp::Node {
public:
  ConnectivityChecker() : Node("connectivity_checker") {
    // Parse environment variables with defaults
    const char *num_drones_env = std::getenv("NUM_DRONES");
    num_drones_ = num_drones_env ? std::stoi(num_drones_env) : 5;

    const char *comm_range_env = std::getenv("COMM_RANGE");
    comm_range_ = comm_range_env ? std::strtod(comm_range_env, nullptr) : 3.0;

    RCLCPP_INFO(
        this->get_logger(),
        "Connectivity Checker initialized with NUM_DRONES=%d, COMM_RANGE=%.2f",
        num_drones_, comm_range_);

    callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_;

    // Subscribe to drone poses
    for (int i = 1; i <= num_drones_; ++i) {
      std::string topic = "/cf_" + std::to_string(i) + "/pose";
      std::string drone_name = "cf_" + std::to_string(i);

      auto callback =
          [this,
           drone_name](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            drone_poses_[drone_name] = msg->pose.position;
          };

      auto sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          topic, 10, callback, sub_opt);
      drone_subs_.push_back(sub);

      RCLCPP_INFO(this->get_logger(), "Subscribed to %s", topic.c_str());
    }

    // Subscribe to AGV pose
    agv_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/AGV/pose", 10,
        [this](const geometry_msgs::msg::Point::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          agv_pose_ = *msg;
          has_agv_pose_ = true;
        },
        sub_opt);

    // Service client for octomap
    client_ = this->create_client<octomap_msgs::srv::GetOctomap>(
        "octomap_binary", rmw_qos_profile_services_default, callback_group_);

    // Timer to fetch octomap once
    init_timer_ = this->create_wall_timer(100ms, [this]() {
      if (map_ready_)
        return;
      fetchOctomapOnce();
    });

    // Main connectivity check timer (5Hz)
    timer_ = this->create_wall_timer(
        200ms, std::bind(&ConnectivityChecker::checkConnectivity, this),
        callback_group_);

    RCLCPP_INFO(this->get_logger(), "Connectivity Checker online");
  }

private:
  int num_drones_;
  double comm_range_;
  std::mutex data_mutex_;

  std::map<std::string, geometry_msgs::msg::Point> drone_poses_;
  geometry_msgs::msg::Point agv_pose_;
  bool has_agv_pose_ = false;

  std::shared_ptr<octomap::OcTree> tree_;
  std::atomic<bool> map_ready_{false};

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr>
      drone_subs_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr agv_sub_;
  rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr timer_;

  void fetchOctomapOnce() {
    if (!client_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "Waiting for octomap service...");
      return;
    }

    auto req = std::make_shared<octomap_msgs::srv::GetOctomap::Request>();
    auto future = client_->async_send_request(req);

    if (future.wait_for(3s) != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get OctoMap");
      return;
    }

    auto res = future.get();
    if (!res)
      return;

    octomap::AbstractOcTree *abs_tree = octomap_msgs::binaryMsgToMap(res->map);
    if (!abs_tree)
      return;

    tree_.reset(dynamic_cast<octomap::OcTree *>(abs_tree));
    if (!tree_)
      return;

    map_ready_ = true;
    RCLCPP_INFO(this->get_logger(), "OctoMap cached successfully");
  }

  double computeDistance(const geometry_msgs::msg::Point &a,
                         const geometry_msgs::msg::Point &b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  bool checkLineOfSight3D(const geometry_msgs::msg::Point &a,
                          const geometry_msgs::msg::Point &b) {
    if (!tree_)
      return false;

    octomap::point3d start(a.x, a.y, a.z);
    octomap::point3d end(b.x, b.y, b.z);
    octomap::point3d direction = end - start;
    octomap::point3d hit;

    // Cast ray from start to end
    // If we hit something before reaching the end point, line of sight is
    // blocked
    bool ray_hit =
        tree_->castRay(start, direction, hit, true, direction.norm());

    if (!ray_hit) {
      // No obstacle hit, line of sight is clear
      return true;
    }

    // Check if hit point is beyond the target (meaning target was reached)
    double dist_to_hit = (hit - start).norm();
    double dist_to_target = (end - start).norm();

    // If we hit something before the target, line of sight is blocked
    // Add small epsilon for numerical stability
    return dist_to_hit >= (dist_to_target - 0.01);
  }

  std::string createJSON(bool connected, double distance,
                         const std::string &entity1, const std::string &entity2,
                         bool is_agv_connection) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "{\"connected\": " << (connected ? "true" : "false")
        << ", \"distance\": " << distance;

    if (is_agv_connection) {
      oss << ", \"drone\": \"" << entity1 << "\", \"agv\": \"" << entity2
          << "\"}";
    } else {
      oss << ", \"drone1\": \"" << entity1 << "\", \"drone2\": \"" << entity2
          << "\"}";
    }

    return oss.str();
  }

  void checkConnectivity() {
    if (!map_ready_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Map not ready yet");
      return;
    }

    if (!has_agv_pose_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "AGV pose not received yet");
      return;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);

    if (drone_poses_.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "No drone poses received yet");
      return;
    }

    // Track connection count per drone
    std::map<std::string, int> connection_count;
    for (const auto &[drone_name, _] : drone_poses_) {
      connection_count[drone_name] = 0;
    }

    // Find nearest drone to AGV
    std::string nearest_to_agv;
    double min_agv_dist = std::numeric_limits<double>::max();
    bool agv_connection_valid = false;

    for (const auto &[drone_name, drone_pos] : drone_poses_) {
      double dist = computeDistance(drone_pos, agv_pose_);
      if (dist < min_agv_dist) {
        min_agv_dist = dist;
        nearest_to_agv = drone_name;
      }
    }

    // Check if nearest drone to AGV is actually connected
    if (!nearest_to_agv.empty()) {
      bool los = checkLineOfSight3D(drone_poses_[nearest_to_agv], agv_pose_);
      agv_connection_valid = (min_agv_dist <= comm_range_) && los;

      if (agv_connection_valid) {
        connection_count[nearest_to_agv]++;
      }

      std::string json = createJSON(agv_connection_valid, min_agv_dist,
                                    nearest_to_agv, "AGV", true);
      RCLCPP_INFO(this->get_logger(), "%s", json.c_str());
    }

    // For each drone, find its nearest neighbor drone
    std::vector<std::string> drone_names;
    for (const auto &[name, _] : drone_poses_) {
      drone_names.push_back(name);
    }

    std::set<std::pair<std::string, std::string>> printed_pairs;

    for (const auto &drone1 : drone_names) {
      std::string nearest_neighbor;
      double min_dist = std::numeric_limits<double>::max();

      for (const auto &drone2 : drone_names) {
        if (drone1 == drone2)
          continue;

        double dist =
            computeDistance(drone_poses_[drone1], drone_poses_[drone2]);
        if (dist < min_dist) {
          min_dist = dist;
          nearest_neighbor = drone2;
        }
      }

      if (!nearest_neighbor.empty()) {
        // Create a sorted pair to avoid duplicates
        std::string first =
            (drone1 < nearest_neighbor) ? drone1 : nearest_neighbor;
        std::string second =
            (drone1 < nearest_neighbor) ? nearest_neighbor : drone1;
        std::pair<std::string, std::string> pair_key = {first, second};

        // Only print each pair once
        if (printed_pairs.find(pair_key) == printed_pairs.end()) {
          printed_pairs.insert(pair_key);

          bool los = checkLineOfSight3D(drone_poses_[drone1],
                                        drone_poses_[nearest_neighbor]);
          bool connected = (min_dist <= comm_range_) && los;

          if (connected) {
            connection_count[drone1]++;
            connection_count[nearest_neighbor]++;
          }

          std::string json =
              createJSON(connected, min_dist, drone1, nearest_neighbor, false);
          RCLCPP_INFO(this->get_logger(), "%s", json.c_str());
        }
      }
    }

    // Output summary of isolated drones
    std::vector<std::string> isolated_drones;
    for (const auto &[drone_name, count] : connection_count) {
      if (count == 0) {
        isolated_drones.push_back(drone_name);
      }
    }

    if (!isolated_drones.empty()) {
      std::ostringstream oss;
      oss << "ISOLATED DRONES: [";
      for (size_t i = 0; i < isolated_drones.size(); ++i) {
        oss << "\"" << isolated_drones[i] << "\"";
        if (i < isolated_drones.size() - 1) {
          oss << ", ";
        }
      }
      oss << "]";
      RCLCPP_WARN(this->get_logger(), "%s", oss.str().c_str());
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "All drones are connected to the network");
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ConnectivityChecker>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
