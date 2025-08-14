#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <vector>
#include <mutex>

namespace path_follower_pkg {

struct VehicleState {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double velocity = 0.0;
    bool valid = false;
};

struct ControllerConfig {
    // Pure Pursuit parameters
    double lookahead_distance = 2.0;
    double min_lookahead = 1.0;
    double max_lookahead = 5.0;
    double lookahead_ratio = 0.2;  // lookahead = velocity * ratio
    
    // Vehicle parameters
    double wheelbase = 0.33;
    double max_steering_angle = 0.4;  // radians (~23 degrees)
    
    // Speed control
    double target_speed = 3.0;
    double max_speed = 6.0;
    double min_speed = 1.0;
    double speed_lookahead_gain = 0.5;
    
    // Corner handling
    double corner_detection_angle = 0.3;
    double corner_speed_factor = 0.4;
    double anticipation_distance = 2.0;
    
    // Safety parameters
    double max_lateral_error = 2.0;
    double emergency_brake_distance = 0.5;
};

class PathFollower : public rclcpp::Node {
public:
    PathFollower();
    virtual ~PathFollower();

private:
    bool initialize();
    
    // Callbacks
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void control_timer_callback();
    
    // Pure Pursuit Controller
    std::pair<double, double> pure_pursuit_control();
    int find_target_point(const nav_msgs::msg::Path& path, const VehicleState& vehicle, bool in_corner = false);
    double calculate_lookahead_distance(double velocity, bool in_corner = false);
    double calculate_steering_angle(double target_x, double target_y, const VehicleState& vehicle);
    double calculate_target_speed(double steering_angle, double lateral_error, bool in_corner = false);
    
    // Corner detection and handling
    bool detect_upcoming_corner(const nav_msgs::msg::Path& path, const VehicleState& vehicle);
    double calculate_path_curvature(const nav_msgs::msg::Path& path, int start_idx, int samples = 5);
    double predict_required_steering(const nav_msgs::msg::Path& path, const VehicleState& vehicle);
    
    // Utility functions
    double normalize_angle(double angle);
    double distance_to_path(const nav_msgs::msg::Path& path, const VehicleState& vehicle);
    bool is_path_valid(const nav_msgs::msg::Path& path);
    
    // ROS components
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // State variables
    nav_msgs::msg::Path current_path_;
    VehicleState vehicle_state_;
    ControllerConfig config_;
    
    // Thread safety
    std::mutex path_mutex_;
    std::mutex state_mutex_;
    
    // Control variables
    rclcpp::Time last_path_time_;
    bool path_received_;
    bool emergency_stop_;
    int last_target_index_;
};

} // namespace path_follower_pkg