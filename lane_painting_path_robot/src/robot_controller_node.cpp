#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class RobotControllerNode : public rclcpp::Node {
public:
    RobotControllerNode() : Node("robot_controller") {
        // Subscribe to odometry for robot pose
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&RobotControllerNode::odomCallback, this, std::placeholders::_1));

        // Subscribe to lane detection commands
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&RobotControllerNode::cmdCallback, this, std::placeholders::_1));

        // Create publisher for velocity commands to Gazebo
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/model/vehicle_blue/cmd_vel", 10);

        // Create timer for control loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // 20Hz control loop
            std::bind(&RobotControllerNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Robot controller node initialized");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_ = msg;
        
        // Extract yaw from quaternion
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_yaw_ = yaw;
    }

    void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        target_linear_vel_ = msg->linear.x;
        target_angular_vel_ = msg->angular.z;
        last_cmd_time_ = this->now();
        has_received_cmd_ = true;
    }

    void controlLoop() {
        if (!current_pose_) {
            return;
        }

        // Check if we haven't received a command for a while
        auto current_time = this->now();
        if (has_received_cmd_ && (current_time - last_cmd_time_).seconds() > 1.0) {
            // Stop the robot if no recent commands
            target_linear_vel_ = 0.0;
            target_angular_vel_ = 0.0;
        }

        // Create and publish velocity command
        auto cmd_vel = geometry_msgs::msg::Twist();
        
        // Apply smooth acceleration
        current_linear_vel_ = smoothVelocity(current_linear_vel_, target_linear_vel_, max_linear_accel_);
        current_angular_vel_ = smoothVelocity(current_angular_vel_, target_angular_vel_, max_angular_accel_);
        
        cmd_vel.linear.x = current_linear_vel_;
        cmd_vel.angular.z = current_angular_vel_;

        cmd_vel_pub_->publish(cmd_vel);
    }

    double smoothVelocity(double current, double target, double max_change) {
        double dt = 0.05;  // 50ms control loop
        double change = target - current;
        change = std::max(-max_change * dt, std::min(max_change * dt, change));
        return current + change;
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    nav_msgs::msg::Odometry::SharedPtr current_pose_;
    double current_yaw_ = 0.0;
    
    double target_linear_vel_ = 0.0;
    double target_angular_vel_ = 0.0;
    double current_linear_vel_ = 0.0;
    double current_angular_vel_ = 0.0;
    
    const double max_linear_accel_ = 1.0;    // m/s^2
    const double max_angular_accel_ = 2.0;   // rad/s^2
    
    rclcpp::Time last_cmd_time_;
    bool has_received_cmd_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
