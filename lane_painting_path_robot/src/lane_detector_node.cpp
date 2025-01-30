#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

class LaneDetectorNode : public rclcpp::Node {
public:
    LaneDetectorNode() : Node("lane_detector") {
        // Create subscriber for camera images
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&LaneDetectorNode::imageCallback, this, std::placeholders::_1));

        // Create publisher for lane detection visualization
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/lane_detection/image", 10);

        // Create publisher for robot control commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "Lane detector node initialized");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // Convert ROS image to OpenCV image
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat frame = cv_ptr->image;

            // Process image for lane detection
            cv::Mat processed_frame = processFrame(frame);

            // Publish processed image
            sensor_msgs::msg::Image::SharedPtr output_msg = 
                cv_bridge::CvImage(msg->header, "bgr8", processed_frame).toImageMsg();
            image_pub_->publish(*output_msg);

            // Calculate steering command based on lane detection
            double steering = calculateSteering(processed_frame);
            publishCommand(steering);
        }
        catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
    }

    cv::Mat processFrame(const cv::Mat& frame) {
        cv::Mat processed;
        
        // Convert to grayscale
        cv::cvtColor(frame, processed, cv::COLOR_BGR2GRAY);

        // Apply Gaussian blur
        cv::GaussianBlur(processed, processed, cv::Size(5, 5), 0);

        // Apply Canny edge detection
        cv::Canny(processed, processed, 50, 150);

        // Define region of interest
        cv::Mat mask = cv::Mat::zeros(processed.size(), CV_8UC1);
        std::vector<cv::Point> roi_points;
        roi_points.push_back(cv::Point(0, processed.rows));
        roi_points.push_back(cv::Point(processed.cols, processed.rows));
        roi_points.push_back(cv::Point(processed.cols/2, processed.rows/2));
        cv::fillConvexPoly(mask, roi_points, cv::Scalar(255));

        // Apply mask
        cv::bitwise_and(processed, mask, processed);

        // Detect lines using Hough transform
        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(processed, lines, 1, CV_PI/180, 50, 50, 10);

        // Draw detected lines
        cv::Mat color_frame;
        cv::cvtColor(processed, color_frame, cv::COLOR_GRAY2BGR);
        for(size_t i = 0; i < lines.size(); i++) {
            cv::Vec4i l = lines[i];
            cv::line(color_frame, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3);
        }

        return color_frame;
    }

    double calculateSteering(const cv::Mat& processed_frame) {
        // Simple steering calculation based on the position of detected lines
        // This is a basic implementation and can be improved
        double steering = 0.0;
        // Add your steering calculation logic here
        return steering;
    }

    void publishCommand(double steering) {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 0.2;  // Constant forward velocity
        msg.angular.z = steering;
        cmd_vel_pub_->publish(msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaneDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
