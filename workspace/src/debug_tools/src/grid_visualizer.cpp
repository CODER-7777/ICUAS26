#include <cmath>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class GridVisualizer : public rclcpp::Node {
public:
  GridVisualizer() : Node("grid_visualizer") {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/grid_markers", 10);
    grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/reward_grid", 10,
        std::bind(&GridVisualizer::gridCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "GridVisualizer listening on /reward_grid");
  }

private:
  void gridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    visualization_msgs::msg::MarkerArray array;
    // Don't create a marker for every cell, it's too heavy (e.g. 10k markers).
    // Let's employ a strategy: only show non-zero or specific range?
    // Or generic "Voxels".

    visualization_msgs::msg::Marker points;
    points.header = msg->header;
    points.header.frame_id = "world";
    points.ns = "grid_points";
    points.id = 0;
    points.type = visualization_msgs::msg::Marker::CUBE_LIST; // Efficient batch
    points.action = visualization_msgs::msg::Marker::ADD;
    points.scale.x = msg->info.resolution * 0.9;
    points.scale.y = msg->info.resolution * 0.9;
    points.scale.z = 0.1; // Thin slice

    for (size_t i = 0; i < msg->data.size(); ++i) {
      int val = msg->data[i];
      if (val <= 0)
        continue; // Skip empty/low reward (assuming >0 is interest)

      // Map index to x,y
      int x = i % msg->info.width;
      int y = i / msg->info.width;

      geometry_msgs::msg::Point p;
      p.x = msg->info.origin.position.x + (x + 0.5) * msg->info.resolution;
      p.y = msg->info.origin.position.y + (y + 0.5) * msg->info.resolution;
      p.z = 1.0; // Fixed height for vis
      points.points.push_back(p);

      // Color based on value
      std_msgs::msg::ColorRGBA color;
      color.a = 0.8;
      // Gradient Red (low) to Green (high)
      float norm = std::clamp((float)val / 100.0f, 0.0f, 1.0f);
      color.r = 1.0f - norm;
      color.g = norm;
      color.b = 0.0f;
      points.colors.push_back(color);
    }

    if (!points.points.empty()) {
      array.markers.push_back(points);
      marker_pub_->publish(array);
    }
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GridVisualizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
