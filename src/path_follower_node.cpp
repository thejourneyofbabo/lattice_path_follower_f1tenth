#include "path_follower_pkg/path_follower.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<path_follower_pkg::PathFollower>();
    
    RCLCPP_INFO(node->get_logger(), "Starting Path Follower Node");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}