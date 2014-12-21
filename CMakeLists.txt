cmake_minimum_required(VERSION 2.8.3)
project(ribbon_cut)


find_package(catkin
  REQUIRED
  rospy
  sensor_msgs
  std_msgs
  tf
  cv_bridge
  baxter_core_msgs
  baxter_interface
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
  sensor_msgs
  std_msgs
  tf
  cv_bridge
  baxter_core_msgs
  baxter_interface
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS scripts/ribbon_cut scripts/image
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(PROGRAMS scripts/image
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
