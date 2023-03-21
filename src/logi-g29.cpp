#include <cstdio>

#include "logi_g29/logi_g29_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;
    auto node = std::make_shared<LogiG29Node>(options);
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}

