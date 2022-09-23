cmake_minimum_required(VERSION 3.0.2)
project(ring_detect)

find_package(OpenCV)

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

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(ring_detect src/ring_detect.cpp src/bringup.cpp)

target_link_libraries(ring_detect
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)