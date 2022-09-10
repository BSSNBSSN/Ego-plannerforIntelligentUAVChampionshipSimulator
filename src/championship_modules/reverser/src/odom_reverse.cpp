#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <tf/tf.h>

#include <cmath>

class SubPuber
{
private:

  ros::NodeHandle nodeHandle;
  // Initial odom
  ros::Subscriber airsimOdomSub;
  // Reversed odom
  ros::Publisher reversedOdomPub;

public:

  SubPuber(){
    airsimOdomSub = nodeHandle.subscribe("/airsim_node/drone_1/odom_local_ned", 1, &SubPuber::OdomReverseCallback, this);
    reversedOdomPub = nodeHandle.advertise<nav_msgs::Odometry>("/reversed_odom", 1);
  }

  void OdomReverseCallback(const nav_msgs::Odometry &airsimOdom)
  {
    nav_msgs::Odometry reversedOdom;
    reversedOdom.header.frame_id = airsimOdom.header.frame_id;

    reversedOdom.pose.pose.position.x = airsimOdom.pose.pose.position.x;
    reversedOdom.pose.pose.position.y = -airsimOdom.pose.pose.position.y;
    reversedOdom.pose.pose.position.z = -airsimOdom.pose.pose.position.z;

    // Convert the msg quaternion to tf quaternion
    tf::Quaternion airsimQuaternion;
    airsimQuaternion.setX(airsimOdom.pose.pose.orientation.x);
    airsimQuaternion.setY(airsimOdom.pose.pose.orientation.y);
    airsimQuaternion.setZ(airsimOdom.pose.pose.orientation.z);
    airsimQuaternion.setW(airsimOdom.pose.pose.orientation.w);
    // Get RPY
    tf::Matrix3x3 airsimTransform(airsimQuaternion);
    double roll, pitch, yaw;
    airsimTransform.getRPY(roll, pitch, yaw);
    // Reverse
    tf::Quaternion reversedQuaternion;
    reversedQuaternion.setRPY(roll, -pitch, -yaw);
    reversedOdom.pose.pose.orientation.x = reversedQuaternion.getX();
    reversedOdom.pose.pose.orientation.y = reversedQuaternion.getY();
    reversedOdom.pose.pose.orientation.z = reversedQuaternion.getZ();
    reversedOdom.pose.pose.orientation.w = reversedQuaternion.getW();

    // TODO: Reverse the twist part

    reversedOdomPub.publish(reversedOdom);
  }
};

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "odom_reverse");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubPuber odomReverse;

  ros::spin();

  return 0;
}
