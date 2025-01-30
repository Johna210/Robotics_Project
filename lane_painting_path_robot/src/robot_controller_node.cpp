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

        // Create publisher for velocity commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Create timer for control loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RobotControllerNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Robot controller node initialized");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_ = msg;  
    }

    void controlLoop() {
        if (!current_pose_) {
            return;
        }

        // Implement your control logic here
        // This is a simple example that makes the robot move forward
        auto cmd_vel = geometry_msgs::msg::Twist();
        cmd_vel.linear.x = 0.2;  // Move forward at 0.2 m/s
        cmd_vel.angular.z = 0.0; // No rotation

        cmd_vel_pub_->publish(cmd_vel);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::Odometry::SharedPtr current_pose_;  
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
