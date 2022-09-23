#ifndef RING_DETECT_H
#define RING_DETECT_H

#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>

#define ELEMENT_SIZE 1

using namespace std;
using namespace cv;

class SubPuber
{
private:

  ros::NodeHandle nodeHandle;
  // Image from airsim front RGB camera
  ros::Subscriber imgSub;
  // Detected ring image
  ros::Publisher ringDstFramePub;
  // Detected ring position
  ros::Publisher ringPositionPub;

  Mat ImgProcess(Mat rawimage);
  Mat PNP(const Mat image);

public:

  SubPuber(){
    imgSub = nodeHandle.subscribe("/airsim_node/drone_1/front_center/Scene", 1, &SubPuber::RingDetectCallback, this);
    ringDstFramePub = nodeHandle.advertise<sensor_msgs::Image>("/ring_dst_frame", 1);
    ringPositionPub = nodeHandle.advertise<geometry_msgs::PoseStamped>("/ring_position", 1);
  }

  void RingDetectCallback(const sensor_msgs::ImageConstPtr &frontRGBImg)
  {
    Mat image;
    try
    {
      image = cv_bridge::toCvShare(frontRGBImg, "bgr8")->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert to image!");
      return;
    }

    image = ImgProcess(image);
    image = PNP(image);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    ringDstFramePub.publish(*msg);
  }
};

#endif