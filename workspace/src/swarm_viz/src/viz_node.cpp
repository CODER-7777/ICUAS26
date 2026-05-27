// swarm_viz: debug visualization for the multi-drone swarm planner.
//
// M1: drone marker spheres + per-drone Path trails.
// M2: comm/LOS graph between every drone pair and the base anchor, computed
//     against the same OccupancyGrid the planner uses (republished on
//     /swarm_viz/occupancy_slice). Edges are colored:
//       green  = within comm range AND grid line-of-sight is clear
//       red    = within comm range BUT line-of-sight is blocked
//       yellow = out of comm range (rendered thinner)
//
// All markers share the "world" frame (same convention as the planner).

#include "swarm_viz/marker_helpers.hpp"
#include "graph_traversal/los.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cmath>
#include <cstdlib>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace {

int getNumRobots() {
    if (const char* p = std::getenv("NUM_ROBOTS")) return std::stoi(p);
    return 5;
}

double getCommRange() {
    if (const char* p = std::getenv("COMM_RANGE")) return std::stod(p);
    return 70.0;
}

}  // namespace

class SwarmVizNode : public rclcpp::Node {
public:
    SwarmVizNode() : Node("swarm_viz") {
        this->declare_parameter<std::string>("frame_id", "world");
        // Visualization is non-load-bearing for the planner; keep the rate
        // low so it doesn't steal CPU from the mission loop. 2 Hz matches a
        // human-readable update cadence and halves serialization vs. 5 Hz.
        this->declare_parameter<double>("publish_rate", 2.0);
        // Long trails serialize a Path of N×trail_length PoseStamped every
        // tick through foxglove_bridge — that's the per-tick hot path. 50
        // gives ~25 s of history at 2 Hz, which is plenty for debugging.
        this->declare_parameter<int>("trail_length", 50);
        this->declare_parameter<double>("drone_diameter", 0.3);
        this->declare_parameter<double>("edge_thickness", 0.05);
        this->declare_parameter<bool>("show_comm_range_spheres", false);

        frame_id_       = this->get_parameter("frame_id").as_string();
        trail_length_   = this->get_parameter("trail_length").as_int();
        drone_diameter_ = this->get_parameter("drone_diameter").as_double();
        edge_thickness_ = this->get_parameter("edge_thickness").as_double();
        show_range_     = this->get_parameter("show_comm_range_spheres").as_bool();

        comm_range_ = getCommRange();
        int n = getNumRobots();
        for (int i = 1; i <= n; ++i) drone_ids_.push_back("cf_" + std::to_string(i));

        // Latched grid + base anchor from the planner.
        auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/swarm_viz/occupancy_slice", latched,
            [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(mu_);
                grid_ = msg;
            });

        base_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/swarm_viz/base_anchor", latched,
            [this](geometry_msgs::msg::PointStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(mu_);
                base_anchor_ = msg->point;
            });

        // Per-drone pose subscriptions and Path publishers.
        // Pose topics fire at ~100 Hz per drone. We still want the latest
        // pose (for the sphere marker), but pushing onto the trail deque
        // every callback is wasteful: with trail_length=50 the trail only
        // holds the last 0.5 s of motion AND we churn 500 deque ops/sec.
        // Throttle trail-recording to ~10 Hz so 50 samples = ~5 s.
        for (const auto& id : drone_ids_) {
            pose_subs_.push_back(this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/" + id + "/pose", 10,
                [this, id](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                    std::lock_guard<std::mutex> lk(mu_);
                    poses_[id] = *msg;
                    rclcpp::Time now = this->now();
                    auto it = last_trail_time_.find(id);
                    if (it == last_trail_time_.end() ||
                        (now - it->second).seconds() >= trail_sample_period_) {
                        auto& trail = trails_[id];
                        trail.push_back(msg->pose.position);
                        while ((int)trail.size() > trail_length_) trail.pop_front();
                        last_trail_time_[id] = now;
                    }
                }));

            trail_pubs_[id] = this->create_publisher<nav_msgs::msg::Path>(
                "/swarm_viz/trail/" + id, 10);
        }

        drones_pub_      = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/swarm_viz/drones", 10);
        comm_graph_pub_  = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/swarm_viz/comm_graph", 10);
        comm_range_pub_  = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/swarm_viz/comm_range_spheres", 10);

        double rate = this->get_parameter("publish_rate").as_double();
        auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(0.1, rate)));
        timer_ = this->create_wall_timer(period, std::bind(&SwarmVizNode::tick, this));

        RCLCPP_INFO(this->get_logger(),
                    "swarm_viz up: %d drones, comm_range=%.1fm, frame=%s",
                    n, comm_range_, frame_id_.c_str());
    }

private:
    void tick() {
        const auto now = this->now();

        // Snapshot state under lock; do the building+publishing outside.
        // publish() can block on intra-process delivery / serialization,
        // and pose callbacks at ~500 Hz combined would otherwise contend
        // for this mutex.
        std::map<std::string, geometry_msgs::msg::PoseStamped> poses_snap;
        std::map<std::string, std::deque<geometry_msgs::msg::Point>> trails_snap;
        std::optional<geometry_msgs::msg::Point> base_snap;
        nav_msgs::msg::OccupancyGrid::SharedPtr grid_snap;
        {
            std::lock_guard<std::mutex> lk(mu_);
            poses_snap = poses_;
            trails_snap = trails_;
            base_snap = base_anchor_;
            grid_snap = grid_;  // shared_ptr bump
        }

        publishDrones(now, poses_snap, base_snap);
        publishTrails(now, trails_snap);
        publishCommGraph(now, poses_snap, base_snap, grid_snap);
        if (show_range_) publishCommRangeSpheres(now, poses_snap);
    }

    void publishDrones(const rclcpp::Time& stamp,
                       const std::map<std::string, geometry_msgs::msg::PoseStamped>& poses,
                       const std::optional<geometry_msgs::msg::Point>& base) {
        visualization_msgs::msg::MarkerArray arr;
        int id = 0;
        for (const auto& did : drone_ids_) {
            auto it = poses.find(did);
            if (it == poses.end()) continue;
            const auto& p = it->second.pose.position;

            arr.markers.push_back(swarm_viz::makeSphere(
                frame_id_, "drones", id++, p, drone_diameter_,
                swarm_viz::makeColor(0.2f, 0.6f, 1.0f, 0.95f), stamp));

            arr.markers.push_back(swarm_viz::makeTextLabel(
                frame_id_, "drone_labels", id++, p, did,
                0.25, swarm_viz::makeColor(1.0f, 1.0f, 1.0f, 1.0f), stamp));
        }
        // Base anchor as a distinct marker so the comm graph terminus is visible.
        if (base) {
            arr.markers.push_back(swarm_viz::makeSphere(
                frame_id_, "drones", id++, *base, drone_diameter_ * 1.4,
                swarm_viz::makeColor(0.9f, 0.5f, 0.1f, 0.9f), stamp));
            arr.markers.push_back(swarm_viz::makeTextLabel(
                frame_id_, "drone_labels", id++, *base, "BASE",
                0.3, swarm_viz::makeColor(1.0f, 1.0f, 1.0f, 1.0f), stamp));
        }
        drones_pub_->publish(arr);
    }

    void publishTrails(const rclcpp::Time& stamp,
                       const std::map<std::string, std::deque<geometry_msgs::msg::Point>>& trails) {
        for (const auto& did : drone_ids_) {
            auto it = trails.find(did);
            if (it == trails.end() || it->second.empty()) continue;
            nav_msgs::msg::Path path;
            path.header.frame_id = frame_id_;
            path.header.stamp = stamp;
            path.poses.reserve(it->second.size());
            for (const auto& p : it->second) {
                geometry_msgs::msg::PoseStamped ps;
                ps.header = path.header;
                ps.pose.position = p;
                ps.pose.orientation.w = 1.0;
                path.poses.push_back(ps);
            }
            trail_pubs_[did]->publish(path);
        }
    }

    // Build a list of all anchor points (drones + base) and emit colored
    // line segments for each unordered pair. We deliberately mirror the
    // planner's anchor logic: every drone pose AND the base count as anchors.
    void publishCommGraph(const rclcpp::Time& stamp,
                          const std::map<std::string, geometry_msgs::msg::PoseStamped>& poses,
                          const std::optional<geometry_msgs::msg::Point>& base,
                          const nav_msgs::msg::OccupancyGrid::SharedPtr& grid) {
        // Points only — the old code allocated a std::string label per
        // anchor every tick that was never read.
        std::vector<geometry_msgs::msg::Point> anchors;
        anchors.reserve(drone_ids_.size() + 1);
        for (const auto& did : drone_ids_) {
            auto it = poses.find(did);
            if (it == poses.end()) continue;
            anchors.push_back(it->second.pose.position);
        }
        if (base) anchors.push_back(*base);
        if (anchors.size() < 2) return;  // nothing to draw

        const auto green  = swarm_viz::makeColor(0.1f, 0.9f, 0.1f, 0.95f);
        const auto red    = swarm_viz::makeColor(1.0f, 0.1f, 0.1f, 0.95f);
        const auto yellow = swarm_viz::makeColor(1.0f, 0.85f, 0.1f, 0.5f);

        auto ok_line       = swarm_viz::makeLineList(frame_id_, "comm_los_ok",    0, edge_thickness_,        stamp);
        auto blocked_line  = swarm_viz::makeLineList(frame_id_, "comm_los_block", 1, edge_thickness_,        stamp);
        auto out_of_range  = swarm_viz::makeLineList(frame_id_, "comm_out_range", 2, edge_thickness_ * 0.4,  stamp);

        const bool have_grid = (grid != nullptr);
        const double comm_range_sq = comm_range_ * comm_range_;

        for (size_t i = 0; i < anchors.size(); ++i) {
            for (size_t j = i + 1; j < anchors.size(); ++j) {
                const auto& a = anchors[i];
                const auto& b = anchors[j];
                double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
                double d2 = dx*dx + dy*dy + dz*dz;

                if (d2 > comm_range_sq) {
                    out_of_range.points.push_back(a);
                    out_of_range.points.push_back(b);
                    out_of_range.colors.push_back(yellow);
                    out_of_range.colors.push_back(yellow);
                    continue;
                }

                bool los = !have_grid ||
                           graph_traversal::hasGridLineOfSightWorld(a.x, a.y, b.x, b.y, *grid);
                auto& target = los ? ok_line : blocked_line;
                const auto& c = los ? green : red;
                target.points.push_back(a);
                target.points.push_back(b);
                target.colors.push_back(c);
                target.colors.push_back(c);
            }
        }

        visualization_msgs::msg::MarkerArray arr;
        arr.markers.push_back(std::move(ok_line));
        arr.markers.push_back(std::move(blocked_line));
        arr.markers.push_back(std::move(out_of_range));
        comm_graph_pub_->publish(arr);
    }

    void publishCommRangeSpheres(const rclcpp::Time& stamp,
                                 const std::map<std::string, geometry_msgs::msg::PoseStamped>& poses) {
        visualization_msgs::msg::MarkerArray arr;
        int id = 0;
        const auto color = swarm_viz::makeColor(0.2f, 0.6f, 1.0f, 0.05f);
        for (const auto& did : drone_ids_) {
            auto it = poses.find(did);
            if (it == poses.end()) continue;
            arr.markers.push_back(swarm_viz::makeSphere(
                frame_id_, "comm_range", id++, it->second.pose.position,
                comm_range_ * 2.0, color, stamp));
        }
        comm_range_pub_->publish(arr);
    }

    std::mutex mu_;
    std::string frame_id_;
    int trail_length_ = 200;
    double drone_diameter_ = 0.3;
    double edge_thickness_ = 0.05;
    double comm_range_ = 70.0;
    bool show_range_ = false;

    std::vector<std::string> drone_ids_;
    std::map<std::string, geometry_msgs::msg::PoseStamped> poses_;
    std::map<std::string, std::deque<geometry_msgs::msg::Point>> trails_;
    std::map<std::string, rclcpp::Time> last_trail_time_;
    const double trail_sample_period_ = 0.1;  // 10 Hz trail samples
    std::optional<geometry_msgs::msg::Point> base_anchor_;
    nav_msgs::msg::OccupancyGrid::SharedPtr grid_;

    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_subs_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr base_sub_;

    std::map<std::string, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> trail_pubs_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr drones_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr comm_graph_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr comm_range_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwarmVizNode>());
    rclcpp::shutdown();
    return 0;
}
