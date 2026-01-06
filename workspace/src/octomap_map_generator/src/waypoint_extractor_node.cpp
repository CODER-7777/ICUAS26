#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <chrono>

// Include our solver logic
#include "graph_solver.hpp"

using namespace std::chrono_literals;

class OctomapExtractor : public rclcpp::Node {
public:
    OctomapExtractor() : Node("octomap_extractor") {
        this->declare_parameter<double>("z_target", 1.0);
        this->declare_parameter<std::string>("service_name", "octomap_binary");
        this->declare_parameter<std::string>("output_csv", "final_path_coordinates.csv");

        std::string service_name = this->get_parameter("service_name").as_string();
        
        // Create Service Client
        client_ = this->create_client<octomap_msgs::srv::GetOctomap>(service_name);
        
        RCLCPP_INFO(this->get_logger(), "Waiting for service: %s", service_name.c_str());
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting. Exiting.");
                return;
            }
        }
        RCLCPP_INFO(this->get_logger(), "Service connected. Requesting map...");

        auto request = std::make_shared<octomap_msgs::srv::GetOctomap::Request>();
        auto result_future = client_->async_send_request(request);

        // Wait for result
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            
            auto response = result_future.get();
            this->processOctomap(response->map);
        
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service");
        }
    }

private:
    rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedPtr client_;
    std::shared_ptr<octomap::OcTree> tree_;

    void processOctomap(const octomap_msgs::msg::Octomap &msg) {
        octomap::AbstractOcTree *abstract_tree = octomap_msgs::binaryMsgToMap(msg);
        
        if (abstract_tree) {
            tree_.reset(dynamic_cast<octomap::OcTree *>(abstract_tree));
            if (tree_) {
                RCLCPP_INFO(this->get_logger(), "Map loaded. Res: %f", tree_->getResolution());
                this->runPipeline();
            } else {
                RCLCPP_ERROR(this->get_logger(), "Could not cast to OcTree");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Could not convert message to AbstractOcTree");
        }
    }

    void runPipeline() {
        double z_target = this->get_parameter("z_target").as_double();
        double res = tree_->getResolution();

        // ==========================================
        // 1. Extract 2D Slice from 3D Map
        // ==========================================
        double minX, minY, minZ, maxX, maxY, maxZ;
        tree_->getMetricMin(minX, minY, minZ);
        tree_->getMetricMax(maxX, maxY, maxZ);

        std::vector<cv::Point> occupied_pixels;
        
        octomap::point3d bbx_min(minX, minY, z_target - res/2.0);
        octomap::point3d bbx_max(maxX, maxY, z_target + res/2.0);

        for (auto it = tree_->begin_leafs_bbx(bbx_min, bbx_max); it != tree_->end_leafs_bbx(); ++it) {
            if (tree_->isNodeOccupied(*it)) {
                int gx = static_cast<int>(std::round(it.getX() / res));
                int gy = static_cast<int>(std::round(it.getY() / res));
                occupied_pixels.emplace_back(gx, gy);
            }
        }

        if (occupied_pixels.empty()) {
            RCLCPP_WARN(this->get_logger(), "No obstacles found at z=%.2f", z_target);
            return;
        }

        // ==========================================
        // 2. Create Image for OpenCV
        // ==========================================
        int minGx = 1e9, maxGx = -1e9, minGy = 1e9, maxGy = -1e9;
        for (const auto& p : occupied_pixels) {
            minGx = std::min(minGx, p.x); maxGx = std::max(maxGx, p.x);
            minGy = std::min(minGy, p.y); maxGy = std::max(maxGy, p.y);
        }

        const int PADDING = 20; 
        const int SCALE = 5; 
        int width = (maxGx - minGx + 1 + 2 * PADDING) * SCALE;
        int height = (maxGy - minGy + 1 + 2 * PADDING) * SCALE;
        int offset_x = minGx - PADDING;
        int offset_y = minGy - PADDING;

        cv::Mat grid = cv::Mat::zeros(height, width, CV_8U);

        for (const auto& p : occupied_pixels) {
            int px = (p.x - offset_x) * SCALE;
            int py = (p.y - offset_y) * SCALE;
            cv::rectangle(grid, cv::Point(px, py), cv::Point(px + SCALE, py + SCALE), cv::Scalar(255), cv::FILLED);
        }

        // ==========================================
        // 3. Extract Building Polygons (FIXED LOGIC)
        // ==========================================
        
        // A. Dilate the WHOLE grid first to merge nearby fragments
        cv::Mat dilated;
        // Size(41, 41) creates the safety margin around the poles
        cv::dilate(grid, dilated, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(41, 41)));

        // B. Find contours on the merged map
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(dilated, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::map<int, Point3D> all_vertices;
        std::map<int, int> vertex_to_graph_id;
        std::vector<Edge> intra_edges;
        std::set<int> all_vertex_ids;
        int global_id = 1;
        int cluster_id = 0; 

        RCLCPP_INFO(this->get_logger(), "Found %zu merged obstacles.", contours.size());

        for (const auto& cnt : contours) {
            cluster_id++; 
            
            std::vector<cv::Point> approx;
            // Epsilon 0.04 reduces corner clumping
            cv::approxPolyDP(cnt, approx, 0.04 * cv::arcLength(cnt, true), true);

            if (approx.size() < 3) continue;

            std::vector<int> building_node_ids;

            // Convert pixels back to metric coords
            for (const auto& pt : approx) {
                double mx = ((double)pt.x / SCALE + offset_x) * res;
                double my = ((double)pt.y / SCALE + offset_y) * res;
                
                all_vertices[global_id] = {mx, my, z_target};
                vertex_to_graph_id[global_id] = cluster_id; 
                all_vertex_ids.insert(global_id);
                building_node_ids.push_back(global_id);
                global_id++;
            }

            // Add perimeter edges (Intra-Building)
            for (size_t k = 0; k < building_node_ids.size(); ++k) {
                int u = building_node_ids[k];
                int v = building_node_ids[(k + 1) % building_node_ids.size()];
                intra_edges.push_back({u, v, get_distance(all_vertices[u], all_vertices[v])});
            }
        }

        // ==========================================
        // 4. Find Bridges
        // ==========================================
        RCLCPP_INFO(this->get_logger(), "Calculating bridges...");
        std::vector<Edge> bridges = find_connecting_bridges(all_vertices, vertex_to_graph_id, tree_);

        // ==========================================
        // 5. Solve Path
        // ==========================================
        RCLCPP_INFO(this->get_logger(), "Solving optimal path...");
        std::vector<int> final_path_ids = solve_chinese_postman_path(intra_edges, bridges, all_vertex_ids);

        // ==========================================
        // 6. VISUALIZATION
        // ==========================================
        RCLCPP_INFO(this->get_logger(), "Generating visualization image...");
        
        cv::Mat vis_img;
        cv::cvtColor(grid, vis_img, cv::COLOR_GRAY2BGR); 

        auto metricToPixel = [&](double mx, double my) -> cv::Point {
            int px = (int)((mx / res - offset_x) * SCALE);
            int py = (int)((my / res - offset_y) * SCALE);
            return cv::Point(px, py);
        };

        // A. Draw Intra-Building Edges (Blue)
        for (const auto& edge : intra_edges) {
            Point3D p1 = all_vertices[edge.u];
            Point3D p2 = all_vertices[edge.v];
            cv::line(vis_img, metricToPixel(p1.x, p1.y), metricToPixel(p2.x, p2.y), cv::Scalar(255, 0, 0), 2);
        }

        // B. Draw Bridges (Green)
        for (const auto& edge : bridges) {
            Point3D p1 = all_vertices[edge.u];
            Point3D p2 = all_vertices[edge.v];
            cv::line(vis_img, metricToPixel(p1.x, p1.y), metricToPixel(p2.x, p2.y), cv::Scalar(0, 255, 0), 2);
        }

        // C. Draw Final Path Trajectory (Red)
        if (!final_path_ids.empty()) {
            for (size_t i = 0; i < final_path_ids.size() - 1; ++i) {
                Point3D p1 = all_vertices[final_path_ids[i]];
                Point3D p2 = all_vertices[final_path_ids[i+1]];
                
                // Line connecting path points
                cv::line(vis_img, metricToPixel(p1.x, p1.y), metricToPixel(p2.x, p2.y), cv::Scalar(0, 0, 255), 2);
                
                // Draw Vertex dot (Yellow)
                cv::circle(vis_img, metricToPixel(p1.x, p1.y), 3, cv::Scalar(0, 255, 255), -1); 
            }
            // Draw Start (Green) and End (Red) Points larger
            Point3D start = all_vertices[final_path_ids[0]];
            Point3D end = all_vertices[final_path_ids.back()];
            cv::circle(vis_img, metricToPixel(start.x, start.y), 6, cv::Scalar(0, 255, 0), -1); 
            cv::circle(vis_img, metricToPixel(end.x, end.y), 6, cv::Scalar(0, 0, 255), -1); 
        }

        // D. Draw Point Order Labels
        if (!final_path_ids.empty()) {
            for (size_t i = 0; i < final_path_ids.size(); ++i) {
                Point3D p = all_vertices[final_path_ids[i]];
                cv::Point pix = metricToPixel(p.x, p.y);
                cv::Point textPos = pix + cv::Point(5, -5);

                // Draw number with a black outline for readability
                cv::putText(vis_img, std::to_string(i), textPos, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,0,0), 2); 
                cv::putText(vis_img, std::to_string(i), textPos, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,255,255), 1); 
            }
        }

        // Save Image
        std::string img_filename = "path_visualization.png";
        cv::imwrite(img_filename, vis_img);
        RCLCPP_INFO(this->get_logger(), "Saved visualization to: %s", img_filename.c_str());

        // ==========================================
        // 7. Write to CSV
        // ==========================================
        std::string filename = this->get_parameter("output_csv").as_string();
        std::ofstream csv(filename);
        if (csv.is_open()) {
            csv << "x,y,z\n";
            for (int id : final_path_ids) {
                const auto& p = all_vertices.at(id);
                csv << p.x << "," << p.y << "," << p.z << "\n";
            }
            csv.close();
            RCLCPP_INFO(this->get_logger(), "Path saved to CSV: %s", filename.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open output file.");
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OctomapExtractor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}