#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <visualization_msgs/msg/marker.hpp>

class LaneDetectorNode : public rclcpp::Node {
public:
    LaneDetectorNode() : Node("lane_painter") {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&LaneDetectorNode::imageCallback, this, std::placeholders::_1));
            
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/processed_image", 10);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/paint_marker", 10);
        
        last_paint_time_ = this->now();
        paint_offset_ = 0.0;
        marker_id_ = 0;
        is_painting_ = true;  // Start painting immediately
        
        // Create initial paint line
        initializePaintLine();
        
        // Timer for constant paint rate
        paint_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10Hz update rate
            std::bind(&LaneDetectorNode::paintTimerCallback, this));
            
        RCLCPP_INFO(this->get_logger(), "Paint robot initialized and painting!");
    }

private:
    void initializePaintLine() {
        paint_line_.header.frame_id = "world";
        paint_line_.ns = "paint_line";
        paint_line_.id = 0;
        paint_line_.type = visualization_msgs::msg::Marker::LINE_STRIP;
        paint_line_.action = visualization_msgs::msg::Marker::ADD;
        paint_line_.pose.orientation.w = 1.0;
        
        // Make line thick and bright red
        paint_line_.scale.x = 0.2;  // 20cm wide line
        paint_line_.color.r = 1.0;
        paint_line_.color.g = 0.0;
        paint_line_.color.b = 0.0;
        paint_line_.color.a = 1.0;
        
        // Never expire
        paint_line_.lifetime = rclcpp::Duration::from_seconds(0);
    }
    
    void paintTimerCallback() {
        if (!is_painting_) {
            RCLCPP_INFO(this->get_logger(), "Not painting currently");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Creating paint marker...");
        
        // Create paint marker
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";  // Use odom frame for global positioning
        marker.header.stamp = this->now();
        marker.ns = "paint";
        marker.id = marker_id_++;
        marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Set mesh resource (use a simple plane mesh)
        marker.mesh_resource = "package://lane_painting_path_robot/meshes/paint_plane.dae";
        marker.mesh_use_embedded_materials = false;
        
        // Paint position - at robot's current position
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.001;  // Just above ground
        
        // Flat on ground
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        
        // Very large paint mark
        marker.scale.x = 3.0;    // 3m long
        marker.scale.y = 1.0;    // 1m wide
        marker.scale.z = 0.01;   // 1cm thick
        
        // Bright red color
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        
        // Make markers permanent
        marker.lifetime = rclcpp::Duration::from_seconds(0);
        
        RCLCPP_INFO(this->get_logger(), "Publishing paint marker with ID: %d", marker_id_);
        marker_pub_->publish(marker);
        
        // Add additional markers at different heights for better visibility
        for(double height = 0.002; height <= 0.01; height += 0.002) {
            marker.id = marker_id_++;
            marker.pose.position.z = height;
            marker.scale.z = 0.002;  // Thinner layers
            RCLCPP_INFO(this->get_logger(), "Publishing additional layer at height %f", height);
            marker_pub_->publish(marker);
        }
    }
    
    cv::Mat processFrame(const cv::Mat& frame) {
        cv::Mat result = frame.clone();
        
        int height = frame.rows;
        int width = frame.cols;
        
        // Draw target line in the center
        cv::line(result, 
                cv::Point(width/2, height), 
                cv::Point(width/2, 0), 
                cv::Scalar(0, 0, 255), 8);  // Extra thick red guide line
        
        // Draw guide lines
        cv::line(result,
                cv::Point(width/2 - 50, height),
                cv::Point(width/2 - 50, 0),
                cv::Scalar(0, 255, 0), 4);
        cv::line(result,
                cv::Point(width/2 + 50, height),
                cv::Point(width/2 + 50, 0),
                cv::Scalar(0, 255, 0), 4);
        
        // Draw paint status
        std::string status = is_painting_ ? "PAINTING" : "NOT PAINTING";
        cv::putText(result, status, cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 1.0, 
                   is_painting_ ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), 
                   2);
        
        auto msg = geometry_msgs::msg::Twist();
        
        // Slow, steady forward movement
        msg.linear.x = 0.1;  // 10cm/s
        
        // Use gyro-like correction to maintain straight line
        msg.angular.z = -0.1 * paint_offset_;
        
        // Limit angular velocity
        msg.angular.z = std::max(-0.1, std::min(0.1, msg.angular.z));
        
        cmd_vel_pub_->publish(msg);
        return result;
    }
    
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat processed_frame = processFrame(cv_ptr->image);
            
            sensor_msgs::msg::Image::SharedPtr output_msg = 
                cv_bridge::CvImage(msg->header, "bgr8", processed_frame).toImageMsg();
            image_pub_->publish(*output_msg);
            
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

private:
    rclcpp::Time last_paint_time_;
    double paint_offset_;
    int marker_id_;
    bool is_painting_;
    rclcpp::TimerBase::SharedPtr paint_timer_;
    visualization_msgs::msg::Marker paint_line_;
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaneDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
