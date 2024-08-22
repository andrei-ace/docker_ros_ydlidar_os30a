cmake_minimum_required(VERSION 3.0.2)
project(robot_dog)

find_package(catkin REQUIRED COMPONENTS
  dm_preview
  yolo_detector_ros
)

catkin_package(
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


