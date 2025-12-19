#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <string>
#include <stdexcept>
#include <cmath>
#include <unordered_set>
#include <fstream>
#include <random>
#include <algorithm>
#include <sstream>
#include <memory>
#include <limits>
#include <iomanip>

using namespace std;
using namespace cv;

struct PointHash {
    size_t operator()(const cv::Point2i& p) const {
        auto hash1 = hash<int>()(p.x);
        auto hash2 = hash<int>()(p.y);
        return hash1 ^ (hash2 << 1);
    }
};

struct VerificationPoint {
    octomap::point3d metric_point;
    cv::Point pixel_point;
};

class OctomapExtractor : public rclcpp::Node {
public:
    OctomapExtractor() : Node("octomap_extractor") {
        this->declare_parameter<double>("z_target", 1.0);
        this->declare_parameter<std::string>("output_filename", "final_waypoints.txt");
        this->declare_parameter<std::string>("service_name", "octomap_binary");

        string service_name = this->get_parameter("service_name").as_string();

        RCLCPP_INFO(this->get_logger(), "Creating Client for service: %s", service_name.c_str());
        rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedPtr client =
            this->create_client<octomap_msgs::srv::GetOctomap>(service_name);

        RCLCPP_INFO(this->get_logger(), "Client Created");

        while (!client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for '%s' service...", service_name.c_str());
        }
        RCLCPP_INFO(this->get_logger(), "Service Available");

        auto request = std::make_shared<octomap_msgs::srv::GetOctomap::Request>();
        auto result = client->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Request sent. Waiting for response...");

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Request Successful. Processing OctoMap...");
            this->processOctomap(result.get()->map);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed or timed out.");
        }
    }

private:
    std::shared_ptr<octomap::OcTree> tree;

    void processOctomap(const octomap_msgs::msg::Octomap &msg) {
        if (msg.binary) {
            octomap::AbstractOcTree *abstract_tree = octomap_msgs::binaryMsgToMap(msg);
            if (abstract_tree) {
                tree.reset(dynamic_cast<octomap::OcTree *>(abstract_tree));
                if (tree) {
                    RCLCPP_INFO(this->get_logger(), "Successfully loaded octomap with resolution: %f", tree->getResolution());
                    this->extractAndSaveWaypoints();
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to cast octomap to OcTree");
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to convert Octomap message to AbstractOcTree");
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Received non-binary octomap, skipping...");
        }
    }

    void extractAndSaveWaypoints() {
        if (!tree) {
            RCLCPP_ERROR(this->get_logger(), "Tree is null. Cannot extract waypoints.");
            return;
        }

        double z_target = this->get_parameter("z_target").as_double();
        string output_filename = this->get_parameter("output_filename").as_string();

        RCLCPP_INFO(this->get_logger(), "Extracting waypoints at z=%.2f", z_target);

        double resolution = tree->getResolution();
        double min_x, min_y, min_z;
        double max_x, max_y, max_z;
        tree->getMetricMin(min_x, min_y, min_z);
        tree->getMetricMax(max_x, max_y, max_z);

        double half_res = resolution / 2.0;
        octomap::point3d slice_min(min_x, min_y, z_target - half_res);
        octomap::point3d slice_max(max_x, max_y, z_target + half_res);

        vector<Point2i> occupied_points;
        for (auto it = tree->begin_leafs_bbx(slice_min, slice_max), end = tree->end_leafs_bbx(); it != end; ++it) {
            if (tree->isNodeOccupied(*it)) {
                double x = it.getX();
                double y = it.getY();

                int gx = static_cast<int>(round(x / resolution));
                int gy = static_cast<int>(round(y / resolution));
                occupied_points.emplace_back(gx, gy);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Total occupied voxels at target height: %zu", occupied_points.size());

        if (occupied_points.empty()) {
            RCLCPP_WARN(this->get_logger(), "No occupied voxels found at this height!");
            return;
        }

        unordered_set<Point2i, PointHash> occupied_set(occupied_points.begin(), occupied_points.end());
        vector<vector<Point2i>> safe_point_per_obstacle;
        safe_point_per_obstacle.reserve(occupied_points.size());
        unordered_set<Point2i, PointHash> all_unique_safe_points;

        for (const auto& obstacle_pt : occupied_points) {
            vector<Point2i> current_safe_points;
            Point2i neighbors[4] = {
                Point2i(obstacle_pt.x, obstacle_pt.y + 1), Point2i(obstacle_pt.x, obstacle_pt.y - 1),
                Point2i(obstacle_pt.x + 1, obstacle_pt.y), Point2i(obstacle_pt.x - 1, obstacle_pt.y)
            };
            for (const auto& neighbor : neighbors) {
                if (occupied_set.count(neighbor) == 0) {
                    current_safe_points.push_back(neighbor);
                    all_unique_safe_points.insert(neighbor);
                }
            }
            safe_point_per_obstacle.push_back(current_safe_points);
        }
        RCLCPP_INFO(this->get_logger(), "Total unique safe points found: %zu", all_unique_safe_points.size());

        int min_gx_orig = std::numeric_limits<int>::max();
        int max_gx_orig = std::numeric_limits<int>::min();
        int min_gy_orig = std::numeric_limits<int>::max();
        int max_gy_orig = std::numeric_limits<int>::min();

        for (const auto& pt : occupied_points) {
            min_gx_orig = std::min(min_gx_orig, pt.x);
            max_gx_orig = std::max(max_gx_orig, pt.x);
            min_gy_orig = std::min(min_gy_orig, pt.y);
            max_gy_orig = std::max(max_gy_orig, pt.y);
        }

        int min_gx_padded = min_gx_orig;
        int max_gx_padded = max_gx_orig;
        int min_gy_padded = min_gy_orig;
        int max_gy_padded = max_gy_orig;

        const int PADDING = 10;
        const int CELL_SIZE = 5;

        min_gx_padded -= PADDING;
        min_gy_padded -= PADDING;
        max_gx_padded += PADDING;
        max_gy_padded += PADDING;

        int img_width = (max_gx_padded - min_gx_padded + 1) * CELL_SIZE;
        int img_height = (max_gy_padded - min_gy_padded + 1) * CELL_SIZE;

        if (img_width <= 0 || img_height <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Computed image dimensions <= 0 (width=%d height=%d). Aborting.", img_width, img_height);
            return;
        }

        Mat map_image_color = Mat::zeros(img_height, img_width, CV_8UC3);
        Mat map_image_binary = Mat::zeros(img_height, img_width, CV_8U);

        for (const auto& pt : occupied_points) {
            int px = (pt.x - min_gx_padded) * CELL_SIZE;
            int py = (pt.y - min_gy_padded) * CELL_SIZE;
            rectangle(map_image_color, Point(px, py), Point(px + CELL_SIZE, py + CELL_SIZE), Scalar(0, 0, 255), FILLED);
            rectangle(map_image_binary, Point(px, py), Point(px + CELL_SIZE, py + CELL_SIZE), Scalar(255), FILLED);
        }

        RCLCPP_INFO(this->get_logger(), "--- Finding Building Outlines ---");

        Mat labels;
        int num_labels = connectedComponents(map_image_binary, labels, 8);
        RCLCPP_INFO(this->get_logger(), "Found %d distinct obstacles (clusters).", num_labels - 1);

        vector<vector<Point>> waypoints_per_building;

        for (int i = 1; i < num_labels; ++i) {
            Mat mask = (labels == i);
            vector<Point> cluster_pixels;
            findNonZero(mask, cluster_pixels);
            if (cluster_pixels.size() < 20) continue;

            Mat dilated_mask;
            Mat element = getStructuringElement(MORPH_RECT, Size(51, 51));
            dilate(mask, dilated_mask, element);

            vector<vector<Point>> contours;
            findContours(dilated_mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

            for (size_t j = 0; j < contours.size(); j++) {
                vector<Point> approx_poly;
                double epsilon = 0.007 * arcLength(contours[j], true);
                approxPolyDP(contours[j], approx_poly, epsilon, true);
                waypoints_per_building.push_back(approx_poly);
                vector<vector<Point>> poly_to_draw = {approx_poly};
                polylines(map_image_color, poly_to_draw, true, Scalar(0, 255, 255), 2);
            }
        }

        RCLCPP_INFO(this->get_logger(), "--- Accessing Polygon Vertices (Your Graph Nodes) ---");

        ofstream waypoints_file(output_filename);
        if (!waypoints_file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Error: Could not open %s for writing!", output_filename.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Saving metric waypoints to %s", output_filename.c_str());
            waypoints_file << "# Waypoints extracted at z=" << z_target << "\n";
            waypoints_file << "# format: x y z\n";
        }

        // Use octomap::point3d to match VerificationPoint.metric_point type
        vector<octomap::point3d> all_metric_waypoints;
        all_metric_waypoints.reserve(waypoints_per_building.size() * 8);

        for (size_t i = 0; i < waypoints_per_building.size(); ++i) {
            if (waypoints_file.is_open()) {
                waypoints_file << "# Building Outline #" << i << "\n";
            }

            const auto& polygon_vertices = waypoints_per_building[i];

            for (size_t j = 0; j < polygon_vertices.size(); ++j) {
                const Point& vertex = polygon_vertices[j];

                double gx = (double)(vertex.x) / CELL_SIZE + min_gx_padded;
                double gy = (double)(vertex.y) / CELL_SIZE + min_gy_padded;

                double x_meters = gx * resolution;
                double y_meters = gy * resolution;

                all_metric_waypoints.emplace_back(x_meters, y_meters, z_target);

                if (waypoints_file.is_open()) {
                    waypoints_file << x_meters << " " << y_meters << " " << z_target << "\n";
                }
            }
        }

        if (waypoints_file.is_open()) {
            waypoints_file.close();
            RCLCPP_INFO(this->get_logger(), "Successfully saved waypoints.");
        }

        RCLCPP_INFO(this->get_logger(), "--- Plotting Metric Waypoints for Verification ---");
        Mat verification_image = map_image_color.clone();
        vector<VerificationPoint> verification_points;

        for (const auto& pt_metric : all_metric_waypoints) {
            double gx_from_metric = pt_metric.x() / resolution;
            double gy_from_metric = pt_metric.y() / resolution;
            int px_verify = static_cast<int>(round((gx_from_metric - min_gx_padded) * CELL_SIZE));
            int py_verify = static_cast<int>(round((gy_from_metric - min_gy_padded) * CELL_SIZE));
            circle(verification_image, Point(px_verify, py_verify), 3, Scalar(0, 255, 0), FILLED);
            verification_points.push_back({pt_metric, Point(px_verify, py_verify)});
        }

        std::random_device rd;
        std::mt19937 g(rd());
        std::shuffle(verification_points.begin(), verification_points.end(), g);

        int num_to_label = std::min(10, (int)verification_points.size());
        RCLCPP_INFO(this->get_logger(), "Labeling %d random waypoints...", num_to_label);

        for (int i = 0; i < num_to_label; ++i) {
            const auto& vp = verification_points[i];
            std::ostringstream ss;
            ss << std::fixed << std::setprecision(2) << "(" << vp.metric_point.x() << ", " << vp.metric_point.y() << ")";
            string label = ss.str();
            Point text_pos = vp.pixel_point + Point(5, -5);
            putText(verification_image, label, text_pos, FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 255), 1, LINE_AA);
        }

        string img_filename = "/root/images/map_visualization_contours_with_waypoints.png";
        imwrite(img_filename, verification_image);
        RCLCPP_INFO(this->get_logger(), "Verification map visualization saved to %s", img_filename.c_str());
        RCLCPP_INFO(this->get_logger(), "Metric waypoints saved to %s", output_filename.c_str());
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting waypoint extractor node...");
    auto node = std::make_shared<OctomapExtractor>();
    RCLCPP_INFO(rclcpp::get_logger("main"), "Waypoint extraction complete. Shutting down.");
    rclcpp::shutdown();
    return 0;
}
