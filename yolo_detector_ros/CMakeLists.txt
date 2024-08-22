cmake_minimum_required(VERSION 3.0.2)
project(yolo_detector_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  detection_msgs
)

catkin_package(
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  src/detect.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)