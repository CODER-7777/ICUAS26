#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

using namespace std::chrono_literals;

class OctomapProcessor : public rclcpp::Node {
public:
  OctomapProcessor() : Node("octomap_processor") {
    // Parameters
    this->declare_parameter("resolution", 0.1);
    this->declare_parameter("reward_sigma", 1.0); // Sigma for Gaussian reward
    this->declare_parameter("obstacle_distance_max",
                            2.0); // Max dist for reward calculation
    this->declare_parameter("flatten_strength",
                            0.1); // How much to flatten per visit
    this->declare_parameter("flatten_sigma", 0.5);     // Spread of flattening
    this->declare_parameter("los_penalty_decay", 0.5); // Fast decay for LOS
    this->declare_parameter("grid_z_min", 0.5);        // Height slice min
    this->declare_parameter("grid_z_max", 1.5);        // Height slice max

    // Clients
    octomap_client_ =
        this->create_client<octomap_msgs::srv::GetOctomap>("/octomap_binary");

    // Publishers
    grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/reward_grid", 10);

    // Subscribers for 5 drones
    drone_poses_.resize(5);
    has_pose_.resize(5, false);
    for (int i = 0; i < 5; ++i) {
      std::string topic = "/cf_" + std::to_string(i + 1) + "/pose";
      std::function<void(const geometry_msgs::msg::PoseStamped::SharedPtr)> cb =
          [this, i](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            this->poseCallback(msg, i);
          };
      pose_subs_.push_back(
          this->create_subscription<geometry_msgs::msg::PoseStamped>(topic, 10,
                                                                     cb));
    }

    // Timer for fetching octomap (once or periodic check?)
    // Assuming static map as agreed, fetch once after a delay to ensure server
    // is up.
    map_fetch_timer_ = this->create_wall_timer(
        2s, std::bind(&OctomapProcessor::fetchOctomap, this));

    // Timer for publishing grid
    process_timer_ = this->create_wall_timer(
        100ms, std::bind(&OctomapProcessor::processGrid, this));

    RCLCPP_INFO(this->get_logger(), "OctomapProcessor started.");
  }

private:
  void fetchOctomap() {
    if (octree_ || waiting_for_map_) {
      return;
    }

    if (!octomap_client_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "Waiting for Octomap service...");
      return;
    }

    auto request = std::make_shared<octomap_msgs::srv::GetOctomap::Request>();
    waiting_for_map_ = true;

    // Asynchronous call with callback
    octomap_client_->async_send_request(
        request,
        std::bind(&OctomapProcessor::mapCallback, this, std::placeholders::_1));
  }

  void mapCallback(
      rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedFuture future) {
    waiting_for_map_ = false;
    try {
      auto response = future.get();
      octomap::AbstractOcTree *tree =
          octomap_msgs::binaryMsgToMap(response->map);
      octree_ = std::shared_ptr<octomap::OcTree>(
          dynamic_cast<octomap::OcTree *>(tree));

      if (octree_) {
        RCLCPP_INFO(this->get_logger(), "Octomap received. Resolution: %f",
                    octree_->getResolution());
        initGrid();
        // Cancel timer since we have the map
        if (map_fetch_timer_)
          map_fetch_timer_->cancel();
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to cast octomap.");
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
    }
  }

  void initGrid() {
    if (!octree_)
      return;

    double min_x, min_y, min_z;
    double max_x, max_y, max_z;
    octree_->getMetricMin(min_x, min_y, min_z);
    octree_->getMetricMax(max_x, max_y, max_z);

    grid_res_ = octree_->getResolution();
    grid_width_ = std::ceil((max_x - min_x) / grid_res_);
    grid_height_ = std::ceil((max_y - min_y) / grid_res_);
    grid_origin_x_ = min_x;
    grid_origin_y_ = min_y;

    long_term_grid_.resize(grid_width_ * grid_height_, 0.0f);
    temp_grid_.resize(grid_width_ * grid_height_, 0.0f);
    final_grid_.resize(grid_width_ * grid_height_, 0);

    RCLCPP_INFO(this->get_logger(), "Grid initialized: %d x %d", grid_width_,
                grid_height_);

    // Compute static reward (long term base)
    computeStaticReward();
  }

  void computeStaticReward() {
    // Iterate over grid, find distance to nearest obstacle
    // This is expensive O(N*M), for now simple approximation or efficient
    // check? Let's iterate over octomap occupied nodes instead and spread
    // reward? Or better: Iterate grid cells, check dist to obstacles. For
    // efficiency in this demo: Iterate every cell. If free, search nearest
    // occupied node within max_dist.

    double max_dist = this->get_parameter("obstacle_distance_max").as_double();
    // double sigma = this->get_parameter("reward_sigma").as_double();
    double sigma = 1.3;
    double z_min = this->get_parameter("grid_z_min").as_double();
    double z_max = this->get_parameter("grid_z_max").as_double();

    for (int y = 0; y < grid_height_; ++y) {
      for (int x = 0; x < grid_width_; ++x) {
        double wx = grid_origin_x_ + (x + 0.5) * grid_res_;
        double wy = grid_origin_y_ + (y + 0.5) * grid_res_;

        // Check if this cell itself is occupied in 3D projection?
        // Just check a few height layers or search?
        // For simplified logic: Check if (wx, wy, z) is occupied for z in
        // range.
        bool occupied = false;
        for (double z = z_min; z <= z_max; z += grid_res_) {
          octomap::OcTreeNode *node = octree_->search(wx, wy, z);
          if (node && octree_->isNodeOccupied(node)) {
            occupied = true;
            break;
          }
        }

        if (occupied) {
          // Obstacle
          long_term_grid_[y * grid_width_ + x] = -100.0f; // Very low
          continue;
        }

        // If not occupied, find distance to nearest obstacle
        // Using getUnknownLeafCenters or just searching.
        // Octomap doesn't have fast NN search built-in for unorganized.
        // Brute force is too slow.
        // Approach: Spread Gaussian FROM occupied nodes.
      }
    }

    // Reset grid to 0
    std::fill(long_term_grid_.begin(), long_term_grid_.end(), 0.0f);

    // Iterate leaves and splat gaussian
    for (auto it = octree_->begin_leafs(), end = octree_->end_leafs();
         it != end; ++it) {
      if (octree_->isNodeOccupied(*it)) {
        double ox = it.getX();
        double oy = it.getY();
        double oz = it.getZ();

        if (oz < z_min || oz > z_max)
          continue;

        // Effect range in grid cells
        int range_cells = std::ceil(max_dist / grid_res_);

        int cx = std::floor((ox - grid_origin_x_) / grid_res_);
        int cy = std::floor((oy - grid_origin_y_) / grid_res_);

        for (int dy = -range_cells; dy <= range_cells; ++dy) {
          for (int dx = -range_cells; dx <= range_cells; ++dx) {
            int nx = cx + dx;
            int ny = cy + dy;

            if (nx >= 0 && nx < grid_width_ && ny >= 0 && ny < grid_height_) {
              double dist = std::sqrt(dx * dx + dy * dy) * grid_res_;
              if (dist > max_dist)
                continue;

              // User wants: highest at fixed distance, decreasing towards/away.
              // Gaussian centered at max_dist/2? Or peaks at some dist?
              // "highest at the fixed distance from the boundary"
              // Let's say peak at max_dist/2
              // Or maybe the user means simply Gaussian OF distance?
              // "highest at the fixed distance ... exponentially decreases as
              // we go towards or away" Let 'fixed_dist' be the peak.
              // Let's interpret "fixed distance" as max_dist/2 for now or close
              // to edge? User says "fixed distance from the edges". Let's
              // assume user wants a safe buffer. Gaussian: R = exp( - (d -
              // d_opt)^2 / (2*sigma^2) )
              double d_opt = 3.0; // Optimal distance from obstacle

              double reward =
                  std::exp(-std::pow(dist - d_opt, 2) / (2 * sigma * sigma));

              // Additive or Max? Usually fields are additive or max.
              // Taking MAX to avoid accumulation artifacts.
              long_term_grid_[ny * grid_width_ + nx] =
                  std::max(long_term_grid_[ny * grid_width_ + nx],
                           (float)reward * 100.0f);
            }
          }
        }

        // Mark obstacle itself as negative
        if (cx >= 0 && cx < grid_width_ && cy >= 0 && cy < grid_height_) {
          long_term_grid_[cy * grid_width_ + cx] = -100.0f;
        }
      }
    }
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg,
                    int drone_idx) {
    if (drone_idx >= 0 && drone_idx < 5) {
      drone_poses_[drone_idx] = *msg;
      has_pose_[drone_idx] = true;
    }
  }

  void processGrid() {
    if (!octree_)
      return;

    double flatten = this->get_parameter("flatten_strength").as_double();
    double flatten_sigma = this->get_parameter("flatten_sigma").as_double();

    // 1. Update Long Term Grid (Flattening) - for ALL drones
    for (int i = 0; i < 5; ++i) {
      if (!has_pose_[i])
        continue;

      double drone_x = drone_poses_[i].pose.position.x;
      double drone_y = drone_poses_[i].pose.position.y;

      // Determine range to check (3 * sigma is enough for Gaussian)
      double range = 3.0 * flatten_sigma;
      int range_cells = std::ceil(range / grid_res_);

      // Current grid position of the drone
      int cx = std::floor((drone_x - grid_origin_x_) / grid_res_);
      int cy = std::floor((drone_y - grid_origin_y_) / grid_res_);

      // Looping over cells which are in the range of the drone
      for (int dy = -range_cells; dy <= range_cells; ++dy) {
        for (int dx = -range_cells; dx <= range_cells; ++dx) {
          int nx = cx + dx;
          int ny = cy + dy;

          if (nx >= 0 && nx < grid_width_ && ny >= 0 && ny < grid_height_) {
            // Calculate real distance
            double cell_x = grid_origin_x_ + (nx + 0.5) * grid_res_;
            double cell_y = grid_origin_y_ + (ny + 0.5) * grid_res_;
            double dist_sq =
                std::pow(cell_x - drone_x, 2) + std::pow(cell_y - drone_y, 2);

            // Gaussian falloff
            double weight =
                std::exp(-dist_sq / (2.0 * flatten_sigma * flatten_sigma));

            // Multiplicative decay
            float &val = long_term_grid_[ny * grid_width_ + nx];
            if (val > 0) {
              // flatten serves as decay rate scale (0..1)
              // If weight is 1 and flatten is 0.1, we multiply by 0.9
              double decay_factor = std::max(0.0, 1.0 - (flatten * weight));
              val *= (float)decay_factor;
            }
          }
        }
      }
    }

    // 2. Compute Temp Grid (LOS) - aggregated
    // Initialize with a low negative value (occluded state)
    // "more and more negative as we move away from end of LOS"
    // Let's assume -50.0 is base occlusion penalty, and we add more penalty for
    // depth. Or just start with -100 (Deeply occluded).
    std::fill(temp_grid_.begin(), temp_grid_.end(), 0.0f);

    // For optimization, we can iterate drones then pixels, or pixels then
    // drones. Since we want max(drone1, drone2...), iterating pixels then
    // drones is easier logic wise but raycasting per pixel per drone is
    // expensive.
    //
    // Optimization: Raycast is expensive.
    // Let's do: For each cell, we check visibility against *each* available
    // drone. temp_grid[cell] = max over drones ( visibility_score(drone, cell)
    // ) visible = 0. occluded = -depth * 10.

    // NOTE: This runs at 10Hz. 5 * Width * Height raycasts might be too slow if
    // grid is large. If grid is 100x100 = 10,000 cells. 5 drones = 50,000
    // raycasts. Octomap raycast is fairly fast but 50k might take >100ms. Let's
    // try it. If slow, we optimize by bounding box or similar.

    for (int y = 0; y < grid_height_; ++y) {
      for (int x = 0; x < grid_width_; ++x) {
        double wx = grid_origin_x_ + (x + 0.5) * grid_res_;
        double wy = grid_origin_y_ + (y + 0.5) * grid_res_;

        // Target point
        // Using average drone height or actual drone height? All drones might
        // be at different Z. We project into 2D grid. The visibility depends on
        // the 3D line. Let's check visibility from drone (X,Y,Z) to target
        // (x,y, Z_averaged? or Z_drone?) If we want to know if the cell is
        // visible "on the ground" or "at flight altitude"? Usually "Map
        // visibility". Let's assume visibility of the *cell center* at *grid
        // slice height*. Or better: visibility of the cell at *drone's* height?
        // Let's use a fixed height for the target cell, e.g., 3.0m (center of
        // interest).
        double target_z = 3.0;
        octomap::point3d end(wx, wy, target_z);

        float max_local_val = -1000.0f; // Very negative start
        bool any_drone_active = false;

        for (int i = 0; i < 5; ++i) {
          if (!has_pose_[i])
            continue;
          any_drone_active = true;

          octomap::point3d origin(drone_poses_[i].pose.position.x,
                                  drone_poses_[i].pose.position.y,
                                  3.0
                                  // drone_poses_[i].pose.position.z
                                  );

          octomap::point3d ray_end;
          // max range 3m
          bool hit = octree_->castRay(origin, end - origin, ray_end, true, 3.0);

          double dist_to_cell = (end - origin).norm();
          double dist_to_hit = (ray_end - origin).norm();

          float val;
          if ((!hit && dist_to_cell<=3.0) || dist_to_hit >= dist_to_cell) {
            val = 20.0f; // Visible: Base value to distinguish from unknown
          } else {
            double depth = dist_to_cell - 3.0;
            val = 20.0f * std::exp(-std::pow(depth, 2) / (2 * 1.1 * 1.1));
            // val = -10.0f * (float)depth; // Occluded
          }
          if (val > max_local_val)
            max_local_val = val;
        }

        if (any_drone_active) {
          temp_grid_[y * grid_width_ + x] = max_local_val;
        } else {
          // No drones, keep default or set to specific
          temp_grid_[y * grid_width_ + x] = -50.0f;
        }
      }
    }

    // 3. Combine and Publish
    auto msg = nav_msgs::msg::OccupancyGrid();
    msg.header.stamp = this->now();
    msg.header.frame_id = "world";
    msg.info.resolution = grid_res_;
    msg.info.width = grid_width_;
    msg.info.height = grid_height_;
    msg.info.origin.position.x = grid_origin_x_;
    msg.info.origin.position.y = grid_origin_y_;
    msg.info.origin.orientation.w = 1.0;

    msg.data.resize(grid_width_ * grid_height_);

    for (size_t i = 0; i < grid_width_ * grid_height_; ++i) {
      float sum = long_term_grid_[i] + temp_grid_[i];
      int val = std::clamp((int)sum, 0, 100);
      msg.data[i] = (int8_t)val;
    }

    grid_pub_->publish(msg);
  }

  rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedPtr octomap_client_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::TimerBase::SharedPtr map_fetch_timer_;
  rclcpp::TimerBase::SharedPtr process_timer_;

  std::shared_ptr<octomap::OcTree> octree_;
  bool waiting_for_map_ = false;

  // Multi-drone state
  std::vector<geometry_msgs::msg::PoseStamped> drone_poses_;
  std::vector<bool> has_pose_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr>
      pose_subs_;

  // Grids
  std::vector<float> long_term_grid_;
  std::vector<float> temp_grid_;
  std::vector<int8_t> final_grid_;

  double grid_res_;
  int grid_width_, grid_height_;
  double grid_origin_x_, grid_origin_y_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OctomapProcessor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
