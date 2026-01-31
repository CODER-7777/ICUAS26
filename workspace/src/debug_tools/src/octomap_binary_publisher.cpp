#include <chrono>
#include <memory>
#include <vector>

#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/srv/get_octomap.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class OctomapBinaryPublisher : public rclcpp::Node {
public:
  OctomapBinaryPublisher() : Node("octomap_binary_publisher") {
    // Create a client to get the binary octomap
    client_ =
        this->create_client<octomap_msgs::srv::GetOctomap>("/octomap_binary");

    // Create a publisher to visualize the octomap
    publisher_ = this->create_publisher<octomap_msgs::msg::Octomap>(
        "octomap_visual", 10);

    // Timer to periodically request the map (e.g., every 5 seconds)
    timer_ = this->create_wall_timer(
        0.5s, std::bind(&OctomapBinaryPublisher::request_and_publish_map, this));

    RCLCPP_INFO(this->get_logger(),
                "Octomap Binary Publisher Node has been started.");
  }

private:
  void request_and_publish_map() {
    if (!client_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(),
                  "Service /octomap_binary not available, waiting...");
      return;
    }

    auto request = std::make_shared<octomap_msgs::srv::GetOctomap::Request>();

    auto future_result = client_->async_send_request(
        request, std::bind(&OctomapBinaryPublisher::handle_service_response,
                           this, std::placeholders::_1));
  }

  void handle_service_response(
      rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedFuture future) {
    try {
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(), "Received octomap binary response.");

      // Publish the map for visualization
      publisher_->publish(response->map);

    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
    }
  }

  rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedPtr client_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctomapBinaryPublisher>());
  rclcpp::shutdown();
  return 0;
}
