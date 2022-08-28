#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <tf/tf.h>
#include <quadrotor_msgs/PositionCommand.h>
#include "airsim_ros_pkgs/VelCmd.h"
#include "airsim_ros_pkgs/PoseCmd.h"
#include "airsim_ros_pkgs/Takeoff.h"
#include "airsim_ros_pkgs/Land.h"
#include "airsim_ros_pkgs/GPSYaw.h"

#include <cmath>

class SubPuber
{
private:

  // ros::NodeHandle nodeHandle;
  // // Initial odom
  // ros::Subscriber airsimOdomSub;
  // // Reversed odom
  // ros::Publisher reversedOdomPub;

  ros::NodeHandle nodeHandle;
  // Initial odom
  ros::Subscriber egoCmdSub;
  // Reversed odom
  ros::Publisher reversedCmdPub;

public:

  SubPuber(){
    // airsimOdomSub = nodeHandle.subscribe("/airsim_node/drone_1/odom_local_ned", 1, &SubPuber::OdomReverseCallback, this);
    // reversedOdomPub = nodeHandle.advertise<nav_msgs::Odometry>("/reversed_odom", 1);
    egoCmdSub = nodeHandle.subscribe("/position_cmd", 1, &SubPuber::CmdReverseCallback, this);
    reversedCmdPub = nodeHandle.advertise<airsim_ros_pkgs::VelCmd>("/airsim_node/drone_1/vel_cmd_body_frame", 1);
  }

  void CmdReverseCallback(const quadrotor_msgs::PositionCommand &egoCmd)
  {
    airsim_ros_pkgs::VelCmd reversedCmd;

    reversedCmd.twist.linear.x = egoCmd.velocity.x;
    reversedCmd.twist.linear.y = -egoCmd.velocity.y;
    reversedCmd.twist.linear.z = -egoCmd.velocity.z;

    reversedCmdPub.publish(reversedCmd);
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
