#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.
 
using namespace std::chrono_literals;
 
class CvImageNode : public rclcpp::Node {
public:
  CvImageNode() : Node("opencv_image_publisher"), count_(0) {
    // Declare parameters
    declare_parameter<std::string>("input_image_topic", "/stereo/left/image_rect_color");
    declare_parameter<std::string>("output_image_topic", "/processed_image");

    // Get parameters
    input_image_topic_ = get_parameter("input_image_topic").as_string();
    output_image_topic_ = get_parameter("output_image_topic").as_string();
    
    // Create a subscriber to the camera image topic
    // Subscriber for input image
    img_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
        input_image_topic_, 10,
        std::bind(&CvImageNode::image_callback, this, std::placeholders::_1)
    );
    
    publisher_ = create_publisher<sensor_msgs::msg::Image>(output_image_topic_, 10);

    RCLCPP_INFO(get_logger(), "Node initialized. Subscribing to: %s", input_image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Publishing processed images to: %s", output_image_topic_.c_str());
  }
 
private:

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      // Convert ROS2 image to OpenCV Mat
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      cv::Mat img_ = cv_ptr->image;

      // Convert to grayscale
      cv::Mat gray_img;
      cv::cvtColor(img_, gray_img, cv::COLOR_BGR2GRAY);

      // Hardcoded parameters for blob detection
      cv::SimpleBlobDetector::Params params;
      params.minThreshold = 0.0;  // Minimum intensity threshold for blob detection
      params.maxThreshold = 255.0; // Maximum intensity threshold

      // Filter by area
      params.filterByArea = true;
      params.minArea = M_PI * pow(sphere_diameter_pixels / 2.0 * 0.8, 2);  // Allow for some tolerance
      params.maxArea = M_PI * pow(sphere_diameter_pixels / 2.0 * 1.2, 2);  // Allow for some tolerance
      // Filter by Circularity
      params.filterByCircularity = true;
      params.minCircularity = 0.785; // Near circular shape
      // Filter by Convexity
      params.filterByConvexity = true;
      params.minConvexity = 0.87;
      // Filter by Inertia
      params.filterByInertia = true;
      params.minInertiaRatio = 0.6; // Tolerance for roundness

      // Set up the detector with default parameters.
      cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

      // Detect blobs
      std::vector<cv::KeyPoint> keypoints;
      detector->detect(gray_img, keypoints);

      // Draw detected blobs as red circles
      // DrawMatchesFlags::DRAW_RICH_KEYPOINTS ensures the size corresponds to blob size
      cv::Mat im_with_keypoints;
      cv::drawKeypoints(gray_img, keypoints, im_with_keypoints, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

      // Convert the processed image to ROS2 message
      auto msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", im_with_keypoints).toImageMsg();

      // Publish the image to the topic defined in the publisher
      publisher_->publish(*msg_);
      RCLCPP_INFO(this->get_logger(), "Image %ld published", count_);
      count_++; 

    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  // Publisher variables
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Image::SharedPtr msg_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  size_t count_;

  // Subscriber variables
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_subscriber_;

  // Params
  std::string input_image_topic_;
  std::string output_image_topic_;

  
  // Hardcoded parameters for sphere detection
  double sphere_diameter = 0.15;  // Diameter of the sphere in real-world units
  double pixels_per_meter = 1000.0;  // Conversion factor from meters to pixels (example value)
  double sphere_diameter_pixels = sphere_diameter * pixels_per_meter;

};
 
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // create a ros2 node
  auto node = std::make_shared<CvImageNode>();
 
  // process ros2 callbacks until receiving a SIGINT (ctrl-c)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}