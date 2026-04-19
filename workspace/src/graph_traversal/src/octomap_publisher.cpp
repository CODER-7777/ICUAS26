#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <octomap_msgs/msg/octomap.hpp>

using namespace std::chrono_literals;

class OctomapServiceToTopic : public rclcpp::Node {
public:
    OctomapServiceToTopic()
        : Node("octomap_service_to_topic") {
        client_ = this->create_client<octomap_msgs::srv::GetOctomap>(
            "octomap_binary");

        pub_ = this->create_publisher<octomap_msgs::msg::Octomap>(
            "octomap_binary_topic", rclcpp::QoS(1).transient_local());

        timer_ = this->create_wall_timer(
            500ms, std::bind(&OctomapServiceToTopic::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Octomap service → topic bridge started");
    }

private:
    void timerCallback() {
        if (!client_->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Waiting for octomap_binary service...");
            return;
        }

        auto request = std::make_shared<octomap_msgs::srv::GetOctomap::Request>();

        client_->async_send_request(
            request,
            [this](rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedFuture future) {
                auto response = future.get();
                if (response) {
                    pub_->publish(response->map);
                }
            });
    }

    rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedPtr client_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OctomapServiceToTopic>());
    rclcpp::shutdown();
    return 0;
}
