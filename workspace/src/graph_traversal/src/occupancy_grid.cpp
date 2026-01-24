#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <octomap/octomap.h>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class OctomapToPlane : public rclcpp::Node {
public:
    OctomapToPlane() : Node("octomap_plane_extractor") {
        this->declare_parameter<double>("z_target", 1.0);
        this->declare_parameter<double>("update_period", 2.0);
        this->declare_parameter<std::string>("frame_id", "map");

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_slice", qos);
        
        client_ = this->create_client<octomap_msgs::srv::GetOctomap>("octomap_binary");

        double period = this->get_parameter("update_period").as_double();
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(period),
            std::bind(&OctomapToPlane::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Extractor Node initialized.");
    }

private:
    void timerCallback() {
        if (!client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Waiting for Octomap service...");
            return;
        }
        auto request = std::make_shared<octomap_msgs::srv::GetOctomap::Request>();
        
        // Corrected async_send_request call for ROS 2 Humble
        client_->async_send_request(request, std::bind(&OctomapToPlane::handleResponse, this, std::placeholders::_1));
    }

    // Corrected SharedFuture type signature
    void handleResponse(rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedFuture future) {
        auto response = future.get();
        if (!response) {
            RCLCPP_ERROR(this->get_logger(), "Service response is null");
            return;
        }

        octomap::AbstractOcTree *abs_tree = octomap_msgs::binaryMsgToMap(response->map);
        if (!abs_tree) return;

        auto tree = std::unique_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree *>(abs_tree));
        double res = tree->getResolution();
        double z_target = this->get_parameter("z_target").as_double();

        double minX, minY, minZ, maxX, maxY, maxZ;
        tree->getMetricMin(minX, minY, minZ);
        tree->getMetricMax(maxX, maxY, maxZ);

        octomap::point3d bbx_min(minX, minY, z_target - res/2.0);
        octomap::point3d bbx_max(maxX, maxY, z_target + res/2.0);

        std::vector<std::pair<int, int>> occupied_points;
        int minGx = 2e9, maxGx = -2e9, minGy = 2e9, maxGy = -2e9;

        for (auto it = tree->begin_leafs_bbx(bbx_min, bbx_max); it != tree->end_leafs_bbx(); ++it) {
            if (tree->isNodeOccupied(*it)) {
                int gx = static_cast<int>(std::floor(it.getX() / res));
                int gy = static_cast<int>(std::floor(it.getY() / res));
                
                occupied_points.push_back({gx, gy});
                
                // Fixed indentation/logic for min/max tracking
                if (gx < minGx) minGx = gx; 
                if (gx > maxGx) maxGx = gx;
                if (gy < minGy) minGy = gy; 
                if (gy > maxGy) maxGy = gy;
            }
        }

        if (occupied_points.empty()) {
            RCLCPP_WARN(this->get_logger(), "No occupied cells at Z=%.2f", z_target);
            return;
        }

        int width = maxGx - minGx + 1;
        int height = maxGy - minGy + 1;

        nav_msgs::msg::OccupancyGrid ros_map;
        ros_map.header.stamp = this->now();
        ros_map.header.frame_id = this->get_parameter("frame_id").as_string();
        ros_map.info.resolution = res;
        ros_map.info.width = width;
        ros_map.info.height = height;
        ros_map.info.origin.position.x = minGx * res;
        ros_map.info.origin.position.y = minGy * res;
        ros_map.info.origin.position.z = z_target;
        ros_map.data.assign(width * height, 0); 

        cv::Mat grid_img = cv::Mat::zeros(height, width, CV_8U);

        for (const auto& pt : occupied_points) {
            int lx = pt.first - minGx;
            int ly = pt.second - minGy;

            int ros_idx = ly * width + lx;
            if (ros_idx >= 0 && ros_idx < static_cast<int>(ros_map.data.size())) {
                ros_map.data[ros_idx] = 100;
                grid_img.at<uchar>(ly, lx) = 255;
            }
        }

        // Output results
        publisher_->publish(ros_map);
        cv::imwrite("latest_plane_slice.png", grid_img);
        RCLCPP_INFO(this->get_logger(), "Published: %d x %d at Z=%.2f", width, height, z_target);
    }

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
    rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OctomapToPlane>());
    rclcpp::shutdown();
    return 0;
}