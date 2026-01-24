#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <octomap/octomap.h>
#include <queue>
#include <vector>
#include <mutex>
#include <fstream>

using namespace std::chrono_literals;

struct idx { int i, j; };

class OctomapBFSPlanner : public rclcpp::Node {
public:
    OctomapBFSPlanner() : Node("octomap_bfs_planner") {
        this->declare_parameter<double>("z_target", 1.0);
        this->declare_parameter<double>("max_dist", 3.0);
        this->declare_parameter<std::string>("csv_name", "path_log.csv");

        // Use a Reentrant group for multi-threading
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_;

        agv_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/AGV/pose", 10, [this](const geometry_msgs::msg::Point::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                latest_agv_pose_ = *msg;
                has_pose_ = true;
            }, sub_opt);

        client_ = this->create_client<octomap_msgs::srv::GetOctomap>("octomap_binary", 
            rmw_qos_profile_services_default, callback_group_);

        // Timer to control processing rate (preventing crashes/hangs)
        timer_ = this->create_wall_timer(500ms, std::bind(&OctomapBFSPlanner::timerLoop, this), callback_group_);

        // Open CSV
        csv_file_.open(this->get_parameter("csv_name").as_string());
        csv_file_ << "timestamp,agv_x,agv_y,drone_points\n";

        start_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Planner initialized. Processing at 2Hz.");
    }

private:
    void timerLoop() {
        double elapsed = (this->now() - start_time_).seconds();
        if (elapsed > 120.0) {
            RCLCPP_INFO_ONCE(this->get_logger(), "120s limit reached. Logging stopped.");
            return;
        }

        if (!has_pose_ || !client_->wait_for_service(100ms)) return;

        auto request = std::make_shared<octomap_msgs::srv::GetOctomap::Request>();
        auto result = client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result, 200ms) == 
            rclcpp::FutureReturnCode::SUCCESS) {
            processPlanning(result.get()->map, elapsed);
        }
    }

    void processPlanning(const octomap_msgs::msg::Octomap& msg, double timestamp) {
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

        // PRINT POINTS
        RCLCPP_INFO(this->get_logger(), "--- [T=%.1f] Points ---", timestamp);
        csv_file_ << timestamp << "," << target.x << "," << target.y << ",";
        
        for (size_t i = 0; i < path.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "P%zu: %.2f, %.2f", i, path[i].x, path[i].y);
            csv_file_ << path[i].x << ";" << path[i].y << (i == path.size() - 1 ? "" : "|");
        }
        csv_file_ << "\n";
        csv_file_.flush();
    }

    std::vector<geometry_msgs::msg::Point> runPlanningPipeline(const nav_msgs::msg::OccupancyGrid& map, geometry_msgs::msg::Point target) {
        double res = map.info.resolution;
        double ox = map.info.origin.position.x;
        double oy = map.info.origin.position.y;

        auto wToG = [&](double x, double y) -> idx {
            return { (int)std::floor((x - ox) / res), (int)std::floor((y - oy) / res) };
        };
        auto gToW = [&](int i, int j) -> geometry_msgs::msg::Point {
            geometry_msgs::msg::Point p;
            p.x = ox + (i + 0.5) * res; p.y = oy + (j + 0.5) * res;
            p.z = this->get_parameter("z_target").as_double();
            return p;
        };

        idx start = wToG(0, 0), end = wToG(target.x, target.y);
        auto grid_path = bfs(start, end, map);
        if (grid_path.empty()) return {};

        std::vector<geometry_msgs::msg::Point> drone_pts;
        idx cur = start;
        double maxD = this->get_parameter("max_dist").as_double();

        while (rclcpp::ok() && !(cur.i == end.i && cur.j == end.j)) {
            bool moved = false;
            for (int i = grid_path.size() - 1; i >= 0; --i) {
                if (grid_path[i].i == cur.i && grid_path[i].j == cur.j) continue;
                auto p1 = gToW(cur.i, cur.j), p2 = gToW(grid_path[i].i, grid_path[i].j);
                double d = std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
                if (d > 0.01 && d < maxD && hasLineOfSight(p1, p2)) {
                    cur = grid_path[i]; drone_pts.push_back(p2); moved = true; break;
                }
            }
            if (!moved) break;
        }
        return drone_pts;
    }

    std::vector<idx> bfs(idx start, idx end, const nav_msgs::msg::OccupancyGrid& map) {
        int w = map.info.width, h = map.info.height;
        if (start.i < 0 || start.i >= w || start.j < 0 || start.j >= h) return {};
        std::queue<idx> q; q.push(start);
        std::vector<int> dist(w * h, -1); std::vector<idx> parent(w * h, {-1, -1});
        dist[start.j * w + start.i] = 0;
        int dx[] = {1, -1, 0, 0, 1, 1, -1, -1}, dy[] = {0, 0, 1, -1, 1, -1, 1, -1};
        while (rclcpp::ok() && !q.empty()) {
            idx cur = q.front(); q.pop();
            if (cur.i == end.i && cur.j == end.j) break;
            for (int k = 0; k < 8; ++k) {
                idx next = {cur.i + dx[k], cur.j + dy[k]};
                if (next.i >= 0 && next.i < w && next.j >= 0 && next.j < h) {
                    int iv = next.j * w + next.i;
                    if (map.data[iv] == 0 && dist[iv] == -1) {
                        dist[iv] = dist[cur.j * w + cur.i] + 1; parent[iv] = cur; q.push(next);
                    }
                }
            }
        }
        if (dist[end.j * w + end.i] == -1) return {};
        std::vector<idx> p;
        for (idx c = end; c.i != -1; c = parent[c.j * w + c.i]) p.push_back(c);
        std::reverse(p.begin(), p.end());
        return p;
    }

    bool hasLineOfSight(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
        octomap::point3d origin(a.x, a.y, a.z), end(b.x, b.y, b.z), hit;
        return !tree_->castRay(origin, (end - origin).normalize(), hit, true, origin.distance(end));
    }

    nav_msgs::msg::OccupancyGrid generateOccupancyGrid() {
        double res = tree_->getResolution(), z = this->get_parameter("z_target").as_double();
        double minX, minY, minZ, maxX, maxY, maxZ;
        tree_->getMetricMin(minX, minY, minZ); tree_->getMetricMax(maxX, maxY, maxZ);
        int minGx = std::floor(minX / res), minGy = std::floor(minY / res);
        int w = std::ceil(maxX / res) - minGx + 1, h = std::ceil(maxY / res) - minGy + 1;
        nav_msgs::msg::OccupancyGrid grid; grid.info.resolution = res;
        grid.info.width = w; grid.info.height = h;
        grid.info.origin.position.x = minGx * res; grid.info.origin.position.y = minGy * res;
        grid.data.assign(w * h, 0);
        for (auto it = tree_->begin_leafs_bbx({(float)minX, (float)minY, (float)(z - res/2)}, {(float)maxX, (float)maxY, (float)(z + res/2)}); it != tree_->end_leafs_bbx(); ++it) {
            if (tree_->isNodeOccupied(*it)) {
                int lx = std::floor(it.getX() / res) - minGx, ly = std::floor(it.getY() / res) - minGy;
                if (lx >= 0 && lx < w && ly >= 0 && ly < h) grid.data[ly * w + lx] = 100;
            }
        }
        return grid;
    }

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr agv_sub_;
    rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    std::shared_ptr<octomap::OcTree> tree_;
    geometry_msgs::msg::Point latest_agv_pose_;
    bool has_pose_ = false;
    std::mutex data_mutex_;
    std::ofstream csv_file_;
    rclcpp::Time start_time_;
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