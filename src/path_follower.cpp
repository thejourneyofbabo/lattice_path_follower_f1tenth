#include "path_follower_pkg/path_follower.hpp"

namespace path_follower_pkg {

PathFollower::PathFollower() 
    : Node("path_follower"),
      path_received_(false),
      emergency_stop_(false),
      last_target_index_(0),
      shutdown_requested_(false),
      has_velocity_path_(false),
      smoothed_velocity_(0.0) {
    
    if (!initialize()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize path follower");
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Path Follower initialized successfully");
}

PathFollower::~PathFollower() {
    // Ensure car stops when node is destroyed
    publish_stop_command();
}

bool PathFollower::initialize() {
    // Declare parameters
    this->declare_parameter("lookahead_distance", config_.lookahead_distance);
    this->declare_parameter("min_lookahead", config_.min_lookahead);
    this->declare_parameter("max_lookahead", config_.max_lookahead);
    this->declare_parameter("lookahead_ratio", config_.lookahead_ratio);
    this->declare_parameter("wheelbase", config_.wheelbase);
    this->declare_parameter("max_steering_angle", config_.max_steering_angle);
    this->declare_parameter("target_speed", config_.target_speed);
    this->declare_parameter("max_speed", config_.max_speed);
    this->declare_parameter("min_speed", config_.min_speed);
    this->declare_parameter("corner_detection_angle", config_.corner_detection_angle);
    this->declare_parameter("corner_speed_factor", config_.corner_speed_factor);
    this->declare_parameter("anticipation_distance", config_.anticipation_distance);
    this->declare_parameter("use_planner_velocity", config_.use_planner_velocity);
    this->declare_parameter("velocity_scale_factor", config_.velocity_scale_factor);
    this->declare_parameter("velocity_smoothing", config_.velocity_smoothing);
    
    // Get parameters
    config_.lookahead_distance = this->get_parameter("lookahead_distance").as_double();
    config_.min_lookahead = this->get_parameter("min_lookahead").as_double();
    config_.max_lookahead = this->get_parameter("max_lookahead").as_double();
    config_.lookahead_ratio = this->get_parameter("lookahead_ratio").as_double();
    config_.wheelbase = this->get_parameter("wheelbase").as_double();
    config_.max_steering_angle = this->get_parameter("max_steering_angle").as_double();
    config_.target_speed = this->get_parameter("target_speed").as_double();
    config_.max_speed = this->get_parameter("max_speed").as_double();
    config_.min_speed = this->get_parameter("min_speed").as_double();
    config_.corner_detection_angle = this->get_parameter("corner_detection_angle").as_double();
    config_.corner_speed_factor = this->get_parameter("corner_speed_factor").as_double();
    config_.anticipation_distance = this->get_parameter("anticipation_distance").as_double();
    config_.use_planner_velocity = this->get_parameter("use_planner_velocity").as_bool();
    config_.velocity_scale_factor = this->get_parameter("velocity_scale_factor").as_double();
    config_.velocity_smoothing = this->get_parameter("velocity_smoothing").as_double();
    
    // Initialize publishers
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        "/drive", 10);
    
    // Initialize subscribers  
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/planned_path", 10,
        std::bind(&PathFollower::path_callback, this, std::placeholders::_1));
        
    path_with_velocity_sub_ = this->create_subscription<planning_custom_msgs::msg::PathWithVelocity>(
        "/planned_path_with_velocity", 10,
        std::bind(&PathFollower::path_with_velocity_callback, this, std::placeholders::_1));
        
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&PathFollower::odom_callback, this, std::placeholders::_1));
    
    // Initialize control timer (50 Hz)
    auto timer_period = std::chrono::milliseconds(20);
    control_timer_ = this->create_wall_timer(
        timer_period, std::bind(&PathFollower::control_timer_callback, this));
    
    last_path_time_ = this->get_clock()->now();
    
    return true;
}

void PathFollower::path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (!is_path_valid(*msg)) {
        RCLCPP_WARN(this->get_logger(), "Received invalid path");
        return;
    }
    
    std::lock_guard<std::mutex> lock(path_mutex_);
    current_path_ = *msg;
    path_received_ = true;
    last_path_time_ = this->get_clock()->now();
    last_target_index_ = 0;  // Reset target index for new path
    
    RCLCPP_DEBUG(this->get_logger(), "Received new path with %zu points", 
                 current_path_.poses.size());
}

void PathFollower::path_with_velocity_callback(const planning_custom_msgs::msg::PathWithVelocity::SharedPtr msg) {
    if (msg->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty velocity path");
        return;
    }
    
    std::lock_guard<std::mutex> lock(path_mutex_);
    current_velocity_path_ = *msg;
    has_velocity_path_ = true;
    path_received_ = true;
    last_path_time_ = this->get_clock()->now();
    last_target_index_ = 0;  // Reset target index for new path
    
    RCLCPP_DEBUG(this->get_logger(), "Received new velocity path with %zu points, velocity_scale_factor=%.2f", 
                 current_velocity_path_.points.size(), config_.velocity_scale_factor);
}

void PathFollower::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    vehicle_state_.x = msg->pose.pose.position.x;
    vehicle_state_.y = msg->pose.pose.position.y;
    
    // Convert quaternion to yaw
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    vehicle_state_.yaw = yaw;
    
    // Calculate velocity
    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    vehicle_state_.velocity = std::sqrt(vx*vx + vy*vy);
    
    vehicle_state_.valid = true;
}

void PathFollower::control_timer_callback() {
    // Check if shutdown was requested
    if (shutdown_requested_.load()) {
        publish_stop_command();
        return;
    }
    
    if (!vehicle_state_.valid || !path_received_) {
        // Send stop command if no valid state or path
        publish_stop_command();
        return;
    }
    
    // Check if path is too old (emergency stop after 2 seconds)
    auto current_time = this->get_clock()->now();
    auto path_age = (current_time - last_path_time_).seconds();
    if (path_age > 2.0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 
                             1000, "Path is too old, stopping vehicle");
        emergency_stop_ = true;
    } else {
        emergency_stop_ = false;
    }
    
    if (emergency_stop_) {
        publish_stop_command();
        return;
    }
    
    // Calculate control commands
    auto [steering_angle, speed] = pure_pursuit_control();
    
    // Create and publish drive message
    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    drive_msg.header.stamp = current_time;
    drive_msg.drive.steering_angle = steering_angle;
    drive_msg.drive.speed = speed;
    
    drive_pub_->publish(drive_msg);
    
    RCLCPP_DEBUG(this->get_logger(), "Control: steering=%.3f, speed=%.3f", 
                 steering_angle, speed);
}

std::pair<double, double> PathFollower::pure_pursuit_control() {
    nav_msgs::msg::Path path;
    VehicleState vehicle;
    
    {
        std::lock_guard<std::mutex> path_lock(path_mutex_);
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        path = current_path_;
        vehicle = vehicle_state_;
    }
    
    if (path.poses.empty()) {
        return std::make_pair(0.0, 0.0);
    }
    
    // Detect upcoming corner
    bool in_corner = detect_upcoming_corner(path, vehicle);
    
    // Find target point (with shorter lookahead for corners)
    int target_index = find_target_point(path, vehicle, in_corner);
    if (target_index < 0 || target_index >= static_cast<int>(path.poses.size())) {
        RCLCPP_WARN(this->get_logger(), "No valid target point found");
        return std::make_pair(0.0, 0.0);
    }
    
    // Get target point coordinates
    double target_x = path.poses[target_index].pose.position.x;
    double target_y = path.poses[target_index].pose.position.y;
    
    // Calculate steering angle with hybrid approach
    double pure_pursuit_angle = calculate_steering_angle(target_x, target_y, vehicle);
    
    // HYBRID CONTROL: Add cross-track error correction (Stanley-like)
    double cross_track_correction = calculate_cross_track_correction(path, vehicle, target_index);
    double steering_angle = pure_pursuit_angle + cross_track_correction;
    
    RCLCPP_DEBUG(this->get_logger(), "Steering: PP=%.3f, CT=%.3f, Total=%.3f", 
                 pure_pursuit_angle, cross_track_correction, steering_angle);
    
    // Calculate lateral error for speed adjustment
    double lateral_error = distance_to_path(path, vehicle);
    
    // Get velocity from planner if available
    double planner_velocity = get_velocity_at_point(target_index);
    
    // DEBUG: Log path following behavior
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "Path Follow: target_idx=%d, lookahead=%.2fm, steering=%.3frad, corner=%s, lat_err=%.2fm", 
        target_index, calculate_lookahead_distance(vehicle.velocity, in_corner), 
        steering_angle, in_corner ? "YES" : "NO", lateral_error);
    
    // Calculate target speed (with corner handling)
    double speed = calculate_target_speed(steering_angle, lateral_error, in_corner, planner_velocity);
    
    if (in_corner) {
        RCLCPP_DEBUG(this->get_logger(), "Corner detected! Reducing speed to %.2f", speed);
    }
    
    // Clamp values
    steering_angle = std::clamp(steering_angle, -config_.max_steering_angle, 
                               config_.max_steering_angle);
    speed = std::clamp(speed, config_.min_speed, config_.max_speed);
    
    return std::make_pair(steering_angle, speed);
}

int PathFollower::find_target_point(const nav_msgs::msg::Path& path, 
                                   const VehicleState& vehicle, bool in_corner) {
    if (path.poses.empty()) return -1;
    
    double lookahead = calculate_lookahead_distance(vehicle.velocity, in_corner);
    
    // Start search from last target index to avoid going backwards
    int start_index = std::max(0, last_target_index_);
    
    // Find closest point first
    double min_distance = std::numeric_limits<double>::max();
    int closest_index = start_index;
    
    for (int i = start_index; i < static_cast<int>(path.poses.size()); ++i) {
        double dx = path.poses[i].pose.position.x - vehicle.x;
        double dy = path.poses[i].pose.position.y - vehicle.y;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        if (distance < min_distance) {
            min_distance = distance;
            closest_index = i;
        }
        
        // If we're moving away, we found the closest point
        if (distance > min_distance + 0.5) break;
    }
    
    // Find lookahead point starting from closest point
    for (int i = closest_index; i < static_cast<int>(path.poses.size()); ++i) {
        double dx = path.poses[i].pose.position.x - vehicle.x;
        double dy = path.poses[i].pose.position.y - vehicle.y;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        if (distance >= lookahead) {
            last_target_index_ = i;
            return i;
        }
    }
    
    // If no lookahead point found, use last point
    last_target_index_ = static_cast<int>(path.poses.size()) - 1;
    return last_target_index_;
}

double PathFollower::calculate_lookahead_distance(double velocity, bool in_corner) {
    double dynamic_lookahead = velocity * config_.lookahead_ratio;
    
    // Slightly reduce lookahead distance in corners for better tracking
    if (in_corner) {
        dynamic_lookahead *= 0.9;  // Small reduction, not aggressive
        RCLCPP_DEBUG(this->get_logger(), "Corner detected: adjusted lookahead %.2fm", dynamic_lookahead);
    }
    
    return std::clamp(dynamic_lookahead, config_.min_lookahead, config_.max_lookahead);
}

double PathFollower::calculate_steering_angle(double target_x, double target_y, 
                                             const VehicleState& vehicle) {
    // Transform target point to vehicle coordinate frame
    double dx = target_x - vehicle.x;
    double dy = target_y - vehicle.y;
    
    // Rotate to vehicle frame
    double cos_yaw = std::cos(-vehicle.yaw);
    double sin_yaw = std::sin(-vehicle.yaw);
    double local_x = dx * cos_yaw - dy * sin_yaw;
    double local_y = dx * sin_yaw + dy * cos_yaw;
    
    // Calculate lookahead distance
    double lookahead_distance = std::sqrt(local_x*local_x + local_y*local_y);
    
    if (lookahead_distance < 0.1) {
        return 0.0;  // Too close, no steering
    }
    
    // Pure pursuit steering angle
    double curvature = 2.0 * local_y / (lookahead_distance * lookahead_distance);
    double steering_angle = std::atan(config_.wheelbase * curvature);
    
    return steering_angle;
}

double PathFollower::calculate_cross_track_correction(const nav_msgs::msg::Path& path, const VehicleState& vehicle, int target_index) {
    if (path.poses.empty() || target_index < 0 || target_index >= static_cast<int>(path.poses.size())) {
        return 0.0;
    }
    
    // Find closest point for cross-track error calculation
    double min_distance = std::numeric_limits<double>::max();
    int closest_index = 0;
    double cross_track_error = 0.0;
    
    // Look in a small window around target index for efficiency
    int start_idx = std::max(0, target_index - 5);
    int end_idx = std::min(static_cast<int>(path.poses.size()) - 1, target_index + 5);
    
    for (int i = start_idx; i <= end_idx; ++i) {
        double dx = path.poses[i].pose.position.x - vehicle.x;
        double dy = path.poses[i].pose.position.y - vehicle.y;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        if (distance < min_distance) {
            min_distance = distance;
            closest_index = i;
        }
    }
    
    if (closest_index < static_cast<int>(path.poses.size()) - 1) {
        // Get path direction at closest point
        double path_x1 = path.poses[closest_index].pose.position.x;
        double path_y1 = path.poses[closest_index].pose.position.y;
        double path_x2 = path.poses[closest_index + 1].pose.position.x;
        double path_y2 = path.poses[closest_index + 1].pose.position.y;
        
        double path_dx = path_x2 - path_x1;
        double path_dy = path_y2 - path_y1;
        double path_length = std::sqrt(path_dx*path_dx + path_dy*path_dy);
        
        if (path_length > 1e-6) {
            // Normalize path direction
            path_dx /= path_length;
            path_dy /= path_length;
            
            // Calculate cross-track error (signed distance)
            double vehicle_to_path_x = vehicle.x - path_x1;
            double vehicle_to_path_y = vehicle.y - path_y1;
            
            // Cross product gives signed distance
            cross_track_error = vehicle_to_path_x * (-path_dy) + vehicle_to_path_y * path_dx;
            
            // Stanley-like correction gain (proportional to cross-track error)
            double k_cross_track = 0.5; // Tuning parameter
            double correction = -k_cross_track * std::atan(cross_track_error / (vehicle.velocity + 0.1));
            
            // Limit correction to prevent oscillation
            correction = std::clamp(correction, -0.2, 0.2); // ±0.2 rad limit
            
            RCLCPP_DEBUG(this->get_logger(), "Cross-track: error=%.3fm, correction=%.3frad", 
                         cross_track_error, correction);
            
            return correction;
        }
    }
    
    return 0.0;
}

double PathFollower::calculate_target_speed(double steering_angle, double lateral_error, bool in_corner, double planner_velocity) {
    double base_speed;
    
    // Use planner velocity if available and enabled
    if (config_.use_planner_velocity && planner_velocity > 0.0) {
        // Scale and smooth planner velocity
        double scaled_velocity = planner_velocity * config_.velocity_scale_factor;
        
        // Smooth velocity changes
        if (smoothed_velocity_ > 0.0) {
            smoothed_velocity_ = smoothed_velocity_ * (1.0 - config_.velocity_smoothing) + 
                               scaled_velocity * config_.velocity_smoothing;
        } else {
            smoothed_velocity_ = scaled_velocity;
        }
        
        base_speed = smoothed_velocity_;
        
        RCLCPP_DEBUG(this->get_logger(), "Using planner velocity: raw=%.2f, scaled=%.2f, smoothed=%.2f", 
                     planner_velocity, scaled_velocity, base_speed);
    } else {
        // Fallback to configured target speed
        base_speed = config_.target_speed;
        smoothed_velocity_ = base_speed;  // Update smoothed velocity for consistency
    }
    
    // Base speed reduction based on steering angle (less aggressive when using planner velocity)
    double abs_steering = std::abs(steering_angle);
    double steering_factor = config_.use_planner_velocity && planner_velocity > 0.0 ? 
                            1.0 - (abs_steering / config_.max_steering_angle) * 0.3 :  // Less aggressive with planner
                            1.0 - (abs_steering / config_.max_steering_angle) * 0.8;   // More aggressive fallback
    
    // Reduce speed based on lateral error
    double error_factor = 1.0 - std::min(lateral_error / config_.max_lateral_error, 1.0) * 0.4;
    
    double target_speed = base_speed * steering_factor * error_factor;
    
    // Apply corner speed reduction
    if (in_corner) {
        target_speed *= config_.corner_speed_factor;  // Very aggressive corner slowdown
    }
    
    return std::max(target_speed, config_.min_speed);
}

double PathFollower::normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double PathFollower::distance_to_path(const nav_msgs::msg::Path& path, 
                                     const VehicleState& vehicle) {
    if (path.poses.empty()) return 0.0;
    
    double min_distance = std::numeric_limits<double>::max();
    
    for (const auto& pose : path.poses) {
        double dx = pose.pose.position.x - vehicle.x;
        double dy = pose.pose.position.y - vehicle.y;
        double distance = std::sqrt(dx*dx + dy*dy);
        min_distance = std::min(min_distance, distance);
    }
    
    return min_distance;
}

bool PathFollower::is_path_valid(const nav_msgs::msg::Path& path) {
    if (path.poses.empty()) {
        return false;
    }
    
    if (path.poses.size() < 2) {
        return false;  // Need at least 2 points
    }
    
    return true;
}

double PathFollower::get_velocity_at_point(int target_index) const {
    if (!has_velocity_path_ || target_index < 0 || 
        target_index >= static_cast<int>(current_velocity_path_.points.size())) {
        return -1.0; // Invalid velocity
    }
    
    return current_velocity_path_.points[target_index].velocity;
}

bool PathFollower::detect_upcoming_corner(const nav_msgs::msg::Path& path, const VehicleState& vehicle) {
    if (path.poses.size() < 5) {
        return false;
    }
    
    // Find closest point on path
    double min_distance = std::numeric_limits<double>::max();
    int closest_idx = 0;
    
    for (int i = 0; i < static_cast<int>(path.poses.size()); ++i) {
        double dx = path.poses[i].pose.position.x - vehicle.x;
        double dy = path.poses[i].pose.position.y - vehicle.y;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        if (distance < min_distance) {
            min_distance = distance;
            closest_idx = i;
        }
    }
    
    // Look ahead on the path to detect corners
    int anticipation_points = static_cast<int>(config_.anticipation_distance / 0.2);  // Assuming 0.2m spacing
    int check_start = std::min(closest_idx, static_cast<int>(path.poses.size()) - 5);
    int check_end = std::min(check_start + anticipation_points, static_cast<int>(path.poses.size()) - 1);
    
    // Calculate curvature in the upcoming path segment
    double max_curvature = calculate_path_curvature(path, check_start, check_end - check_start);
    
    // Also check current steering requirement
    double predicted_steering = predict_required_steering(path, vehicle);
    
    // Corner detected if either curvature is high or predicted steering is large
    bool high_curvature = max_curvature > 0.5;  // 0.5 rad/m curvature threshold
    bool high_steering = std::abs(predicted_steering) > config_.corner_detection_angle;
    
    return high_curvature || high_steering;
}

double PathFollower::calculate_path_curvature(const nav_msgs::msg::Path& path, int start_idx, int samples) {
    if (start_idx + samples >= static_cast<int>(path.poses.size()) || samples < 3) {
        return 0.0;
    }
    
    double max_curvature = 0.0;
    
    for (int i = start_idx + 1; i < start_idx + samples - 1; ++i) {
        // Get three consecutive points
        double x1 = path.poses[i-1].pose.position.x;
        double y1 = path.poses[i-1].pose.position.y;
        double x2 = path.poses[i].pose.position.x;
        double y2 = path.poses[i].pose.position.y;
        double x3 = path.poses[i+1].pose.position.x;
        double y3 = path.poses[i+1].pose.position.y;
        
        // Calculate curvature using the circumscribed circle method
        double a = std::sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
        double b = std::sqrt((x3-x2)*(x3-x2) + (y3-y2)*(y3-y2));
        double c = std::sqrt((x3-x1)*(x3-x1) + (y3-y1)*(y3-y1));
        
        if (a < 1e-6 || b < 1e-6 || c < 1e-6) continue;
        
        double s = (a + b + c) / 2.0;
        double area = std::sqrt(s * (s-a) * (s-b) * (s-c));
        
        if (area < 1e-6) continue;
        
        double radius = (a * b * c) / (4.0 * area);
        double curvature = 1.0 / radius;
        
        max_curvature = std::max(max_curvature, curvature);
    }
    
    return max_curvature;
}

double PathFollower::predict_required_steering(const nav_msgs::msg::Path& path, const VehicleState& vehicle) {
    if (path.poses.size() < 2) {
        return 0.0;
    }
    
    // Find a point ahead on the path
    double lookahead = config_.anticipation_distance;
    
    for (const auto& pose : path.poses) {
        double dx = pose.pose.position.x - vehicle.x;
        double dy = pose.pose.position.y - vehicle.y;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        if (distance >= lookahead) {
            // Calculate what steering angle would be required to reach this point
            double cos_yaw = std::cos(-vehicle.yaw);
            double sin_yaw = std::sin(-vehicle.yaw);
            double local_x = dx * cos_yaw - dy * sin_yaw;
            double local_y = dx * sin_yaw + dy * cos_yaw;
            
            if (local_x > 0.1) {  // Avoid division by zero
                double curvature = 2.0 * local_y / (distance * distance);
                return std::atan(config_.wheelbase * curvature);
            }
            break;
        }
    }
    
    return 0.0;
}

void PathFollower::publish_stop_command() {
    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    drive_msg.header.stamp = this->get_clock()->now();
    drive_msg.header.frame_id = "base_link";
    drive_msg.drive.speed = 0.0;
    drive_msg.drive.steering_angle = 0.0;
    drive_msg.drive.acceleration = -5.0;  // Emergency brake
    drive_msg.drive.jerk = 0.0;
    drive_msg.drive.steering_angle_velocity = 0.0;
    
    drive_pub_->publish(drive_msg);
    
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Publishing stop command - Vehicle stopped for safety");
}

void PathFollower::shutdown_handler() {
    RCLCPP_WARN(this->get_logger(), "Shutdown signal received - stopping vehicle safely");
    shutdown_requested_.store(true);
    
    // Publish multiple stop commands to ensure vehicle stops
    for (int i = 0; i < 5; ++i) {
        publish_stop_command();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

} // namespace path_follower_pkg
