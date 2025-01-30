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

            // Calculate steering command based on lane detection
            double steering = calculateSteering(processed_frame);

            // Publish processed image
            sensor_msgs::msg::Image::SharedPtr output_msg = 
                cv_bridge::CvImage(msg->header, "bgr8", processed_frame).toImageMsg();
            image_pub_->publish(*output_msg);

            // Publish control command
            publishCommand(steering);
        }
        catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
    }

    cv::Mat processFrame(const cv::Mat& frame) {
        cv::Mat processed = frame.clone();
        
        // Convert to HSV for better yellow and white line detection
        cv::Mat hsv;
        cv::cvtColor(processed, hsv, cv::COLOR_BGR2HSV);

        // Define color ranges for yellow and white
        cv::Scalar yellow_low(20, 100, 100);
        cv::Scalar yellow_high(30, 255, 255);
        cv::Scalar white_low(0, 0, 200);
        cv::Scalar white_high(180, 30, 255);

        // Create masks for yellow and white colors
        cv::Mat yellow_mask, white_mask;
        cv::inRange(hsv, yellow_low, yellow_high, yellow_mask);
        cv::inRange(hsv, white_low, white_high, white_mask);

        // Combine masks
        cv::Mat color_mask;
        cv::bitwise_or(yellow_mask, white_mask, color_mask);

        // Apply the mask to the original image
        cv::Mat masked;
        cv::bitwise_and(processed, processed, masked, color_mask);

        // Convert to grayscale
        cv::Mat gray;
        cv::cvtColor(masked, gray, cv::COLOR_BGR2GRAY);

        // Apply Gaussian blur
        cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);

        // Apply Canny edge detection
        cv::Mat edges;
        cv::Canny(gray, edges, 50, 150);

        // Define region of interest (ROI)
        cv::Mat mask = cv::Mat::zeros(edges.size(), CV_8UC1);
        std::vector<cv::Point> roi_points;
        roi_points.push_back(cv::Point(0, edges.rows));
        roi_points.push_back(cv::Point(edges.cols, edges.rows));
        roi_points.push_back(cv::Point(edges.cols * 3/4, edges.rows/2));
        roi_points.push_back(cv::Point(edges.cols/4, edges.rows/2));
        cv::fillConvexPoly(mask, roi_points, cv::Scalar(255));

        // Apply ROI mask
        cv::Mat masked_edges;
        cv::bitwise_and(edges, mask, masked_edges);

        // Find lines using Hough transform
        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(masked_edges, lines, 1, CV_PI/180, 50, 50, 10);

        // Separate left and right lines
        std::vector<cv::Vec4i> left_lines, right_lines;
        for(const auto& line : lines) {
            cv::Vec4i l = line;
            double slope = (l[3] - l[1]) / static_cast<double>(l[2] - l[0]);
            
            if(std::abs(slope) < 0.1) continue; // Skip horizontal lines
            
            if(slope < 0) {
                left_lines.push_back(l);
            } else {
                right_lines.push_back(l);
            }
        }

        // Draw detected lines on the original frame
        cv::Mat result = frame.clone();
        drawLines(result, left_lines, cv::Scalar(255, 0, 0));   // Left lines in blue
        drawLines(result, right_lines, cv::Scalar(0, 0, 255));  // Right lines in red

        // Store line information for steering calculation
        left_line_x_ = getAverageX(left_lines);
        right_line_x_ = getAverageX(right_lines);

        return result;
    }

    void drawLines(cv::Mat& image, const std::vector<cv::Vec4i>& lines, const cv::Scalar& color) {
        for(const auto& line : lines) {
            cv::line(image, cv::Point(line[0], line[1]), 
                    cv::Point(line[2], line[3]), color, 2);
        }
    }

    double getAverageX(const std::vector<cv::Vec4i>& lines) {
        if(lines.empty()) return -1;
        
        double sum_x = 0;
        for(const auto& line : lines) {
            sum_x += (line[0] + line[2]) / 2.0;
        }
        return sum_x / lines.size();
    }

    double calculateSteering(const cv::Mat& processed_frame) {
        if(left_line_x_ < 0 && right_line_x_ < 0) {
            return 0.0; // No lines detected
        }

        // Calculate the center point between detected lines
        double center_x;
        if(left_line_x_ < 0) {
            center_x = right_line_x_ - processed_frame.cols/4;
        } else if(right_line_x_ < 0) {
            center_x = left_line_x_ + processed_frame.cols/4;
        } else {
            center_x = (left_line_x_ + right_line_x_) / 2;
        }

        // Calculate error from the center of the image
        double error = center_x - processed_frame.cols/2;
        
        // Apply proportional control
        double kp = 0.002; // Proportional gain
        double steering = -kp * error;
        
        // Limit steering angle
        steering = std::max(-0.5, std::min(0.5, steering));
        
        return steering;
    }

    void publishCommand(double steering) {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 0.5;  // Forward velocity
        msg.angular.z = steering;
        cmd_vel_pub_->publish(msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    double left_line_x_ = -1;
    double right_line_x_ = -1;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaneDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
