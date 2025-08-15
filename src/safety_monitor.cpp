#include "path_follower_pkg/safety_monitor.hpp"

namespace path_follower_pkg {

SafetyMonitor::SafetyMonitor() 
    : Node("safety_monitor"),
      path_follower_alive_(false),
      emergency_stop_active_(false),
      node_check_timeout_(2.0),
      drive_message_timeout_(1.0),
      monitored_node_name_("path_follower"),
      drive_topic_("/drive"),
      emergency_drive_topic_("/emergency_stop"),
      monitor_frequency_(10.0) {
    
    if (!initialize()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize safety monitor");
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Safety Monitor initialized successfully");
    RCLCPP_INFO(this->get_logger(), "Monitoring node: %s", monitored_node_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Drive topic: %s", drive_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Node check timeout: %.2f seconds", node_check_timeout_);
    RCLCPP_INFO(this->get_logger(), "Drive message timeout: %.2f seconds", drive_message_timeout_);
}

SafetyMonitor::~SafetyMonitor() {
}

bool SafetyMonitor::initialize() {
    // Declare parameters
    this->declare_parameter("monitored_node_name", monitored_node_name_);
    this->declare_parameter("drive_topic", drive_topic_);
    this->declare_parameter("emergency_drive_topic", emergency_drive_topic_);
    this->declare_parameter("node_check_timeout", node_check_timeout_);
    this->declare_parameter("drive_message_timeout", drive_message_timeout_);
    this->declare_parameter("monitor_frequency", monitor_frequency_);
    
    // Get parameters
    monitored_node_name_ = this->get_parameter("monitored_node_name").as_string();
    drive_topic_ = this->get_parameter("drive_topic").as_string();
    emergency_drive_topic_ = this->get_parameter("emergency_drive_topic").as_string();
    node_check_timeout_ = this->get_parameter("node_check_timeout").as_double();
    drive_message_timeout_ = this->get_parameter("drive_message_timeout").as_double();
    monitor_frequency_ = this->get_parameter("monitor_frequency").as_double();
    
    // Initialize publisher for emergency stop commands with high priority
    emergency_stop_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        emergency_drive_topic_, rclcpp::QoS(10).reliable());
    
    // Subscribe to drive messages to monitor if path_follower is publishing
    drive_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        drive_topic_, 10,
        std::bind(&SafetyMonitor::drive_callback, this, std::placeholders::_1));
    
    // Initialize service client for checking if node exists
    list_params_client_ = this->create_client<rcl_interfaces::srv::ListParameters>(
        "/" + monitored_node_name_ + "/list_parameters");
    
    // Initialize monitor timer
    auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / monitor_frequency_));
    monitor_timer_ = this->create_wall_timer(
        timer_period, std::bind(&SafetyMonitor::monitor_timer_callback, this));
    
    last_drive_message_time_ = this->get_clock()->now();
    
    return true;
}

void SafetyMonitor::drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr /* msg */) {
    last_drive_message_time_ = this->get_clock()->now();
    
    // If we were in emergency stop mode and now receiving drive messages, 
    // the path follower is working again
    if (emergency_stop_active_) {
        RCLCPP_INFO(this->get_logger(), "Path follower recovered - disabling emergency stop");
        emergency_stop_active_ = false;
    }
}

void SafetyMonitor::monitor_timer_callback() {
    bool node_alive = is_path_follower_alive();
    
    // Check if drive messages are being received
    auto current_time = this->get_clock()->now();
    auto drive_msg_age = (current_time - last_drive_message_time_).seconds();
    bool drive_messages_recent = drive_msg_age < drive_message_timeout_;
    
    // Update node alive status
    bool was_alive = path_follower_alive_;
    path_follower_alive_ = node_alive && drive_messages_recent;
    
    // Log status changes
    if (was_alive != path_follower_alive_) {
        if (path_follower_alive_) {
            RCLCPP_INFO(this->get_logger(), "Path follower node is now alive and publishing");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Path follower node is dead or not publishing!");
            RCLCPP_ERROR(this->get_logger(), "  Node exists: %s", node_alive ? "YES" : "NO");
            RCLCPP_ERROR(this->get_logger(), "  Drive messages recent: %s (%.2fs ago)", 
                        drive_messages_recent ? "YES" : "NO", drive_msg_age);
        }
    }
    
    // If path follower is not alive, issue emergency stop
    if (!path_follower_alive_ && !emergency_stop_active_) {
        RCLCPP_ERROR(this->get_logger(), "EMERGENCY STOP ACTIVATED - Path follower node failure detected!");
        emergency_stop_active_ = true;
        publish_emergency_stop();
    } else if (!path_follower_alive_ && emergency_stop_active_) {
        // Continue publishing emergency stop while node is dead
        publish_emergency_stop();
    }
    
    // Debug logging (throttled)
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "Safety Monitor Status - Node alive: %s, Drive msg age: %.2fs, Emergency stop: %s",
                          path_follower_alive_ ? "YES" : "NO", drive_msg_age, 
                          emergency_stop_active_ ? "ACTIVE" : "INACTIVE");
}

bool SafetyMonitor::is_path_follower_alive() {
    if (!list_params_client_->service_is_ready()) {
        // Service not available means node is likely not running
        return false;
    }
    
    auto request = std::make_shared<rcl_interfaces::srv::ListParameters::Request>();
    request->depth = 1;
    
    // Use async call with short timeout to avoid blocking
    auto future = list_params_client_->async_send_request(request);
    
    // Wait for a short time
    if (future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
        auto response = future.get();
        // If we get a response, the node is alive
        return response != nullptr;
    }
    
    // Timeout or no response means node is likely dead
    return false;
}

void SafetyMonitor::publish_emergency_stop() {
    auto emergency_msg = ackermann_msgs::msg::AckermannDriveStamped();
    emergency_msg.header.stamp = this->get_clock()->now();
    emergency_msg.header.frame_id = "base_link";
    emergency_msg.drive.speed = 0.0;
    emergency_msg.drive.steering_angle = 0.0;
    emergency_msg.drive.acceleration = -5.0;  // Emergency brake
    emergency_msg.drive.jerk = 0.0;
    emergency_msg.drive.steering_angle_velocity = 0.0;
    
    emergency_stop_pub_->publish(emergency_msg);
    
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Publishing emergency stop command - speed: 0.0, steering: 0.0");
}

} // namespace path_follower_pkg