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
        line(contourImage, vertices[i], vertices[(i+1)%4], Scalar(255), 2);
    }
    // Mat dst(contourImage.size(), CV_8UC3);
    Mat dst;
    cvtColor(contourImage, dst, CV_GRAY2RGB);
    const Mat cameraMatrix = (Mat_<double>(3, 3) << 268.5118713378906, 0.0, 320.0,
                                                    0.0, 268.5118713378906, 240.0,
                                                    0.0, 0.0, 1.0);
    const Mat distCoeffs = (Mat_<double>(5, 1) << 0, 0, 0, 0, 0);
    vector<Point3f> objectPoints;
    objectPoints.push_back({-0.5, 0.5, 0});
    objectPoints.push_back({0.5, 0.5, 0});
    objectPoints.push_back({0.5, -0.5, 0});
    objectPoints.push_back({-0.5, -0.5, 0});

    vector<Point2f> imagePoints;
    for(int i = 0; i < 4; i++) {
      imagePoints.push_back(vertices[i]);
    }

    Mat rvec, tvec;
    solvePnP(
      objectPoints,    // object points, Nx3 1-channel or 1xN/Nx1 3-channel, N is the number of points. vector<Point3d> can be also passed
      imagePoints,    // corresponding image points, Nx2 1-channel or 1xN/Nx1 2-channel, N is the number of points. vector<Point2d> can be also passed
      cameraMatrix,    // camera intrinsic matrix
      distCoeffs,    // distortion coefficients
      rvec,    // rotation vector
      tvec    // translation vector
      // false,    // used for SOLVEPNP_ITERATIVE. If true, use the provided rvec and tvec as initial approximations, and further optimize them.
      // SOLVEPNP_IPPE // solving method
    );
    drawFrameAxes(dst, cameraMatrix, distCoeffs, rvec, tvec, 2);

    return dst;
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
