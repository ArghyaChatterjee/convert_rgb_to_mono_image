#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class RGBtoMonoConverter
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber rgb_sub1_;
  ros::Subscriber rgb_sub2_;
  ros::Publisher mono_pub1_;
  ros::Publisher mono_pub2_;
  bool first_message_received1_;
  bool first_message_received2_;

public:
  RGBtoMonoConverter() : first_message_received1_(false), first_message_received2_(false)
  {
    // Initialize subscribers
    rgb_sub1_ = nh_.subscribe("left_camera_input_topic", 1, &RGBtoMonoConverter::convertRGB1, this);
    rgb_sub2_ = nh_.subscribe("right_camera_input_topic", 1, &RGBtoMonoConverter::convertRGB2, this);

    // Initialize publishers
    mono_pub1_ = nh_.advertise<sensor_msgs::Image>("left_camera_output_topic", 1);
    mono_pub2_ = nh_.advertise<sensor_msgs::Image>("right_camera_output_topic", 1);

    // ROS Info for initialization
    ROS_INFO("RGB to Mono Converter initialized with:");
    ROS_INFO("left_camera_input_topic: %s", rgb_sub1_.getTopic().c_str());
    ROS_INFO("left_camera_output_topic: %s", mono_pub1_.getTopic().c_str());
    ROS_INFO("right_camera_input_topic: %s", rgb_sub2_.getTopic().c_str());
    ROS_INFO("right_camera_output_topic: %s", mono_pub2_.getTopic().c_str());
  }

  void convertRGB1(const sensor_msgs::ImageConstPtr& msg)
  {
    if (!first_message_received1_)
    {
      ROS_INFO("First message received on left_camera_input_topic. Conversion process started.");
      first_message_received1_ = true;
    }

    try
    {
      // Convert ROS Image to OpenCV Mat
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "rgb8");
      cv::Mat gray_image;
      cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_RGB2GRAY);

      // Convert OpenCV Mat back to ROS Image
      cv_bridge::CvImage out_msg;
      out_msg.header = msg->header;
      out_msg.encoding = "mono8";
      out_msg.image = gray_image;

      // Publish the converted image
      mono_pub1_.publish(out_msg.toImageMsg());
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }

  void convertRGB2(const sensor_msgs::ImageConstPtr& msg)
  {
    if (!first_message_received2_)
    {
      ROS_INFO("First message received on right_camera_input_topic. Conversion process started.");
      first_message_received2_ = true;
    }

    try
    {
      // Convert ROS Image to OpenCV Mat
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "rgb8");
      cv::Mat gray_image;
      cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_RGB2GRAY);

      // Convert OpenCV Mat back to ROS Image
      cv_bridge::CvImage out_msg;
      out_msg.header = msg->header;
      out_msg.encoding = "mono8";
      out_msg.image = gray_image;

      // Publish the converted image
      mono_pub2_.publish(out_msg.toImageMsg());
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rgb_to_mono_converter");

  RGBtoMonoConverter converter;

  ros::spin();

  return 0;
}
