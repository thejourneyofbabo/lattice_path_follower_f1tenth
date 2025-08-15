#pragma once

#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rcl_interfaces/srv/list_parameters.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>

namespace path_follower_pkg {

class SafetyMonitor : public rclcpp::Node {
public:
    SafetyMonitor();
    virtual ~SafetyMonitor();

private:
    bool initialize();
    
    // Monitor functions
    void monitor_timer_callback();
    bool is_path_follower_alive();
    void publish_emergency_stop();
    void drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
    
    // ROS components
    rclcpp::TimerBase::SharedPtr monitor_timer_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr emergency_stop_pub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub_;
    rclcpp::Client<rcl_interfaces::srv::ListParameters>::SharedPtr list_params_client_;
    
    // State variables
    rclcpp::Time last_drive_message_time_;
    bool path_follower_alive_;
    bool emergency_stop_active_;
    double node_check_timeout_;
    double drive_message_timeout_;
    
    // Configuration
    std::string monitored_node_name_;
    std::string drive_topic_;
    std::string emergency_drive_topic_;
    double monitor_frequency_;
};

} // namespace path_follower_pkg