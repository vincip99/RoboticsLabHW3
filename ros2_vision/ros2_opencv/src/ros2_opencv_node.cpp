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
    declare_parameter<std::string>("input_image_topic", "/image_camera");
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

    // Detect the spherical object and publish the processed image
  /*     publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>(output_image_topic_, 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalImagePublisher::timer_callback, this)); */
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
      cv::Mat gray;
      cv::cvtColor(img_, gray, cv::COLOR_BGR2GRAY);

      // Set up the detector with default parameters.
      cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create();

      // Detect blobs
      std::vector<cv::KeyPoint> keypoints;
      detector->detect(gray, keypoints);

      // Draw detected blobs as red circles
      // DrawMatchesFlags::DRAW_RICH_KEYPOINTS ensures the size corresponds to blob size
      cv::Mat im_with_keypoints;
      cv::drawKeypoints(gray, keypoints, im_with_keypoints, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

      // Convert the processed image to ROS2 message
      auto msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", im_with_keypoints).toImageMsg();

      // Publish the image to the topic defined in the publisher
      publisher_->publish(*msg_);
      RCLCPP_INFO(this->get_logger(), "Image %ld published", count_);
      count_++; 

/*       // Detect circular objects using HoughCircles
      std::vector<cv::Vec3f> circles;
      cv::HoughCircles(
          gray, circles, cv::HOUGH_GRADIENT, 1,
          gray.rows / 8,  // Min distance between circles
          100,            // Upper threshold for Canny edge detector
          30,             // Threshold for center detection
          10, 50          // Min and max radius
      );

      // Draw detected circles on the original image
      for (const auto &circle : circles) {
          cv::Point center(cvRound(circle[0]), cvRound(circle[1]));
          int radius = cvRound(circle[2]);
          cv::circle(img_, center, radius, cv::Scalar(0, 255, 0), 2);
          cv::circle(img_, center, 2, cv::Scalar(0, 0, 255), 3);
      }

      // Publish the processed image
      cv_bridge::CvImage out_msg;
      out_msg.header = msg->header;
      out_msg.encoding = sensor_msgs::image_encodings::BGR8;
      out_msg.image = img_;
      publisher_->publish(*out_msg.toImageMsg());
      count_++; */
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  /*   void timer_callback() {
    // Create a new 640x480 image
    //cv::Mat my_image(cv::Size(640, 480), CV_8UC3);
 
    // Generate an image where each pixel is a random color
    //cv::randu(my_image, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
 
    // Write message to be sent. Member function toImageMsg() converts a CvImage
    // into a ROS image message
    msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", my_image)
               .toImageMsg();
 
    // Publish the image to the topic defined in the publisher
    publisher_->publish(*msg_.get());
    RCLCPP_INFO(this->get_logger(), "Image %ld published", count_);
    count_++; 

    // Publish the processed image
    cv_bridge::CvImage out_msg;
    //out_msg.header = msg->header;
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image = img_;
    publisher_->publish(out_msg.toImageMsg()); */
    // Prepare the output message
 /*     cv_bridge::CvImage out_msg;
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image = img_;  // The processed image

    // Publish the image directly (not as a shared_ptr)
    publisher_->publish(*out_msg.toImageMsg());
  }*/

  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Image::SharedPtr msg_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  size_t count_;

  // Subscriber variables
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_subscriber_;

  // Params
  std::string input_image_topic_;
  std::string output_image_topic_;

  // Processed image
  cv::Mat img_;

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