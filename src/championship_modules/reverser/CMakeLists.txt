cmake_minimum_required(VERSION 3.0.2)
project(reverser)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  tf2
  quadrotor_msgs
  airsim_ros_pkgs
  cv_bridge
  image_transport
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(odom_reverse src/odom_reverse.cpp)
add_executable(cmd_reverse src/cmd_reverse.cpp)

target_link_libraries(odom_reverse ${catkin_LIBRARIES})
target_link_libraries(cmd_reverse ${catkin_LIBRARIES})
