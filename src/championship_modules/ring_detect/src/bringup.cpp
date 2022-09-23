#include "ring_detect.h"

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "ring_detect");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubPuber ringDetect;
  ros::spin();

  return 0;
}
