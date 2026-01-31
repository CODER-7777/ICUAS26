#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <octomap/octomap.h>
#include <queue>
#include <vector>
#include <mutex>
#include <fstream>
#include <algorithm>
#include <atomic>

using namespace std::chrono_literals;

struct idx { int i, j; };

class OctomapBFSPlanner : public rclcpp::Node {
public:
    OctomapBFSPlanner() : Node("octomap_bfs_planner") {
        // Parameters
        this->declare_parameter<double>("z_target", 1.0);
        this->declare_parameter<double>("max_dist", 3.0);
        this->declare_parameter<std::string>("csv_name", "path_log.csv");
        this->declare_parameter<std::string>("frame_id", "map");

        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_;

        // Subscribers
        agv_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/AGV/pose", 10, [this](const geometry_msgs::msg::Point::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                latest_agv_pose_ = *msg;
                has_pose_ = true;
            }, sub_opt);

        // Publishers
        path_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/drone/waypoint_array", 10);

        // Service Client
        client_ = this->create_client<octomap_msgs::srv::GetOctomap>("octomap_binary", 
            rmw_qos_profile_services_default, callback_group_);

        // 2Hz Control Loop
        timer_ = this->create_wall_timer(500ms, std::bind(&OctomapBFSPlanner::timerLoop, this), callback_group_);

        csv_file_.open(this->get_parameter("csv_name").as_string());
        if (csv_file_.is_open()) {
            csv_file_ << "timestamp,agv_x,agv_y,drone_points\n";
        }

        is_planning_ = false;
        RCLCPP_INFO(this->get_logger(), "Planner Online. Publishing to /drone/waypoint_array");
    }

private:
    std::atomic<bool> is_planning_; 
    std::mutex data_mutex_;
    geometry_msgs::msg::Point latest_agv_pose_;
    bool has_pose_ = false;
    std::shared_ptr<octomap::OcTree> tree_;
    std::ofstream csv_file_;
    
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr agv_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr path_pub_;
    rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;

    void timerLoop() {
        if (!rclcpp::ok()) return;
        if (is_planning_.exchange(true)) return;

        if (!has_pose_ || !client_->wait_for_service(100ms)) {
            is_planning_ = false;
            return;
        }

        auto request = std::make_shared<octomap_msgs::srv::GetOctomap::Request>();
        auto result_future = client_->async_send_request(request);
        
        if (result_future.wait_for(250ms) == std::future_status::ready) {
            auto response = result_future.get();
            if (response) {
                this->processPlanning(response->map);
            }
        }
        
        is_planning_ = false; 
    }

    void processPlanning(const octomap_msgs::msg::Octomap& msg) {
        octomap::AbstractOcTree* abs_tree = octomap_msgs::binaryMsgToMap(msg);
        if (!abs_tree) return;
        tree_.reset(dynamic_cast<octomap::OcTree*>(abs_tree));

        auto grid = generateOccupancyGrid();
        geometry_msgs::msg::Point target;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            target = latest_agv_pose_;
        }

        auto path = runPlanningPipeline(grid, target);

        if (!path.empty()) {
            publishPoseArray(path);
            logToCSV(path, target);
        }
    }

    void publishPoseArray(const std::vector<geometry_msgs::msg::Point>& path) {
        geometry_msgs::msg::PoseArray array_msg;
        array_msg.header.stamp = this->now();
        array_msg.header.frame_id = "world";

        for (const auto& pt : path) {
            geometry_msgs::msg::Pose p;
            p.position = pt;
            p.orientation.w = 1.0; 
            array_msg.poses.push_back(p);
        }
        path_pub_->publish(array_msg);
    }

    void logToCSV(const std::vector<geometry_msgs::msg::Point>& path, const geometry_msgs::msg::Point& target) {
        if (csv_file_.is_open()) {
            double ts = this->now().seconds();
            csv_file_ << ts << "," << target.x << "," << target.y << ",";
            for (size_t i = 0; i < path.size(); ++i) {
                csv_file_ << path[i].x << ";" << path[i].y << (i == path.size() - 1 ? "" : "|");
            }
            csv_file_ << "\n";
            csv_file_.flush();
        }
    }

    std::vector<idx> bfs(idx start, idx end, const nav_msgs::msg::OccupancyGrid& map) {
        int w = map.info.width, h = map.info.height;
        if (start.i < 0 || start.i >= w || start.j < 0 || start.j >= h ||
            end.i < 0 || end.i >= w || end.j < 0 || end.j >= h) return {};

        std::queue<int> q; 
        int start_idx = start.j * w + start.i;
        int end_idx = end.j * w + end.i;

        q.push(start_idx);
        std::vector<int> parent(w * h, -1);
        std::vector<bool> visited(w * h, false);
        visited[start_idx] = true;

        int dx[] = {1,-1,0,0,1,1,-1,-1}, dy[] = {0,0,1,-1,1,-1,1,-1};

        bool found = false;
        while (!q.empty()) {
            int curr_flat = q.front(); q.pop();
            if (curr_flat == end_idx) { found = true; break; }

            int cx = curr_flat % w, cy = curr_flat / w;
            for (int k = 0; k < 8; ++k) {
                int nx = cx + dx[k], ny = cy + dy[k];
                int n_idx = ny * w + nx;
                if (nx >= 0 && nx < w && ny >= 0 && ny < h && !visited[n_idx] && map.data[n_idx] == 0) {
                    visited[n_idx] = true;
                    parent[n_idx] = curr_flat;
                    q.push(n_idx);
                }
            }
        }
        if (!found) return {};
        std::vector<idx> path;
        for (int curr = end_idx; curr != -1; curr = parent[curr]) {
            path.push_back({curr % w, curr / w});
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    std::vector<geometry_msgs::msg::Point> runPlanningPipeline(const nav_msgs::msg::OccupancyGrid& map, geometry_msgs::msg::Point target) {
        double res = map.info.resolution;
        double ox = map.info.origin.position.x, oy = map.info.origin.position.y;
        auto gToW = [&](int i, int j) {
            geometry_msgs::msg::Point p;
            p.x = ox + (i + 0.5) * res; p.y = oy + (j + 0.5) * res;
            p.z = this->get_parameter("z_target").as_double();
            return p;
        };

        idx start = {(int)std::floor((0.0 - ox) / res), (int)std::floor((0.0 - oy) / res)};
        idx end = {(int)std::floor((target.x - ox) / res), (int)std::floor((target.y - oy) / res)};

        auto grid_path = bfs(start, end, map);
        if (grid_path.empty()) return {};

        std::vector<geometry_msgs::msg::Point> drone_pts;
        double maxD = this->get_parameter("max_dist").as_double();
        size_t path_idx = 0;
        drone_pts.push_back(gToW(grid_path[0].i, grid_path[0].j));

        while (path_idx < grid_path.size() - 1) {
            bool found_jump = false;
            for (size_t j = grid_path.size() - 1; j > path_idx; --j) {
                auto p_start = gToW(grid_path[path_idx].i, grid_path[path_idx].j);
                auto p_check = gToW(grid_path[j].i, grid_path[j].j);
                if (std::hypot(p_start.x - p_check.x, p_start.y - p_check.y) <= maxD && hasLineOfSight(p_start, p_check)) {
                    drone_pts.push_back(p_check);
                    path_idx = j;
                    found_jump = true;
                    break;
                }
            }
            if (!found_jump) {
                path_idx++;
                drone_pts.push_back(gToW(grid_path[path_idx].i, grid_path[path_idx].j));
            }
        }
        return drone_pts;
    }

    bool hasLineOfSight(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
        if (!tree_) return false;
        octomap::point3d origin(a.x, a.y, a.z), end_p(b.x, b.y, b.z), hit;
        octomap::point3d dir = (end_p - origin);
        double dist = dir.norm();
        if (dist < 0.05) return true;
        return !tree_->castRay(origin, dir.normalize(), hit, true, dist);
    }

    nav_msgs::msg::OccupancyGrid generateOccupancyGrid() {
        double res = tree_->getResolution();
        double z = this->get_parameter("z_target").as_double();
        double minX, minY, minZ, maxX, maxY, maxZ;
        tree_->getMetricMin(minX, minY, minZ); tree_->getMetricMax(maxX, maxY, maxZ);
        
        int minGx = std::floor(minX / res), minGy = std::floor(minY / res);
        int w = std::ceil(maxX / res) - minGx + 1, h = std::ceil(maxY / res) - minGy + 1;
        
        nav_msgs::msg::OccupancyGrid grid;
        grid.info.resolution = res; grid.info.width = w; grid.info.height = h;
        grid.info.origin.position.x = minGx * res; grid.info.origin.position.y = minGy * res;
        grid.data.assign(w * h, 0);

        octomap::point3d bbx_min((float)minX, (float)minY, (float)(z - res));
        octomap::point3d bbx_max((float)maxX, (float)maxY, (float)(z + res));

        for (auto it = tree_->begin_leafs_bbx(bbx_min, bbx_max); it != tree_->end_leafs_bbx(); ++it) {
            if (tree_->isNodeOccupied(*it)) {
                int lx = std::floor(it.getX() / res) - minGx, ly = std::floor(it.getY() / res) - minGy;
                if (lx >= 0 && lx < w && ly >= 0 && ly < h) grid.data[ly * w + lx] = 100;
            }
        }
        return grid;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OctomapBFSPlanner>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
} 