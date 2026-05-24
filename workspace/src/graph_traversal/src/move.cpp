#include "swarm_planner.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SwarmPlanner>();

    // Critical for concurrency: Allows Subscriber to run while Timer is planning
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
