#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>

class SubPuber
{
private:

  ros::NodeHandle nodeHandle;
  // Image from airsim front RGB camera
  ros::Subscriber imgSub;
  // Detected ring position
  ros::Publisher ringPositionPub;

public:

  SubPuber(){
    imgSub = nodeHandle.subscribe("/airsim_node/drone_1/front_center/Scene", 1, &SubPuber::RingDetectCallback, this);
    ringPositionPub = nodeHandle.advertise<geometry_msgs::PoseStamped>("/ring_position", 1);
  }

  void RingDetectCallback(const sensor_msgs::ImageConstPtr &frontRGBImg)
  {
    cv::Mat image;
    try
    {
      image = cv_bridge::toCvShare(frontRGBImg, "bgr8")->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert to image!");
      return;
    }
    cv::imshow("",image);
    cv::waitKey(500);
    std::cout << "Image recieved" << std::endl;
    cv::destroyWindow("");
  }
};

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "cmd_reverse");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubPuber cmdReverse;

  ros::spin();

  return 0;
}
