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

  Mat ImgProcess(Mat rawimage) {
    vector<Mat> channels;
    split(rawimage,channels);

    Mat grayImage = Mat(rawimage.rows, rawimage.cols, CV_8UC1);
    grayImage = 1.5 * channels.at(2) - 0.4 * channels.at(1) - 1.10 * channels.at(0);

    Mat dst;
    threshold(grayImage, dst, 150, 255, THRESH_BINARY);
    Mat element = getStructuringElement(MORPH_RECT,
                                                Size(2*ELEMENT_SIZE + 1, 2*ELEMENT_SIZE + 1),
                                                Point(ELEMENT_SIZE, ELEMENT_SIZE));
    erode(dst, dst, element);  //腐蚀
    dilate(dst, dst, element);  //膨胀

    return dst;
  }

  Mat PNP(const Mat image) {


    Mat contourImage;
    image.copyTo(contourImage);

    // 轮廓发现与绘制
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(contourImage, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point());

    int contourIndex = 0;
    for(int i = 1; i < contours.size(); i++) {
      if(contourArea(contours[i]) > contourArea(contours[contourIndex])) {
        contourIndex = i;
      }
       
      //绘制轮廓
      drawContours(contourImage,contours,i,Scalar(255),1,8,hierarchy);
    }

    RotatedRect rRect = minAreaRect(contours[contourIndex]);

    Point2f vertices[4];
    rRect.points(vertices);
    for (int i = 0; i < 4; i++) {
        line(contourImage, vertices[i], vertices[(i+1)%4], Scalar(255,255,255), 2);
    }
    return contourImage;
  }

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

    cvtColor(image, image, CV_GRAY2RGB);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    ringDstFramePub.publish(*msg);
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
