cmake_minimum_required(VERSION 3.0.2)
project(stereo_cam)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  image_transport
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES stereo_cam
#  CATKIN_DEPENDS camera_info_manager cv_bridge image_transport roscpp rospy std_msgs
#  DEPENDS system_lib
)

set(OpenCV_DIR /usr/include/opencv4)
find_package(OpenCV)
if(NOT OpenCV_FOUND)
   find_package(OpenCV2 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()
 
include_directories(
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)

add_executable(stereo src/stereo.cpp)
target_link_libraries(stereo
${catkin_LIBRARIES}
${OpenCV_LIBS}
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)