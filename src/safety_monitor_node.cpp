#include "path_follower_pkg/safety_monitor.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<path_follower_pkg::SafetyMonitor>();
    
    RCLCPP_INFO(node->get_logger(), "Starting Safety Monitor Node");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}