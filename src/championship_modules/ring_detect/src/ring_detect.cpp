#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>

#define ELEMENT_SIZE 2

using namespace std;
using namespace cv;

class SubPuber
{
private:

  ros::NodeHandle nodeHandle;
  // Image from airsim front RGB camera
  ros::Subscriber imgSub;
  // Detected ring image
  ros::Publisher ringFramePub;
  // Detected ring position
  ros::Publisher ringPositionPub;

  Mat ImgProcess(Mat rawimage) {
    vector<Mat> channels;
    split(rawimage,channels);

    Mat grayImage = Mat(rawimage.rows, rawimage.cols, CV_8UC1);
    grayImage = 1.5 * channels.at(2) - 0.4 * channels.at(1) - 1.10 * channels.at(0);

    Mat dst;
    threshold(grayImage, dst, 170, 255, THRESH_BINARY);
    Mat element = getStructuringElement(MORPH_RECT,
                                                Size(2*ELEMENT_SIZE + 1, 2*ELEMENT_SIZE + 1),
                                                Point(ELEMENT_SIZE, ELEMENT_SIZE));
    erode(dst, dst, element);  //腐蚀
    dilate(dst, dst, element);  //膨胀

    // imshow("test0", dst);
    // waitKey(9999);
    return dst;
  }


public:

  SubPuber(){
    imgSub = nodeHandle.subscribe("/airsim_node/drone_1/front_center/Scene", 1, &SubPuber::RingDetectCallback, this);
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

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    ringFramePub.publish(*msg);
  }
};

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "ring_detect");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubPuber ringDetect;

  ros::spin();

  return 0;
}
