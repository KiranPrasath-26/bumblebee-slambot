cmake_minimum_required(VERSION 3.0.2)
project(bumblebee_localisation)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  sensor_msgs
)


catkin_python_setup()

catkin_package(
 CATKIN_DEPENDS 
 roscpp 
 rospy 
 nav_msgs
 sensor_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
catkin_install_python(
  PROGRAMS
  nodes/kalman_filter_node.py
  nodes/imu_republisher_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)