cmake_minimum_required(VERSION 3.0.2)
project(stereo_cam)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  image_transport
  roscpp
  rospy
  std_msgs
  message_filters
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES stereo_cam
#  CATKIN_DEPENDS camera_info_manager cv_bridge image_transport roscpp rospy std_msgs
#  DEPENDS system_lib
)

find_package(VPI REQUIRED)  # 查找 VPI 包

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
${VPI_INCLUDE_DIRS}
)

add_executable(stereo src/stereo.cpp)
target_link_libraries(stereo
${catkin_LIBRARIES}
${OpenCV_LIBS}
)

# add_executable(stereo_depth_BM src/stereo_depth_BM.cpp)
# target_link_libraries(stereo_depth_BM
# ${catkin_LIBRARIES}
# ${OpenCV_LIBS}
# ${VPI_LIBRARIES}
# )

add_executable(stereo_depth_SGBM src/stereo_depth_SGBM.cpp)
target_link_libraries(stereo_depth_SGBM
${catkin_LIBRARIES}
${OpenCV_LIBS}
)

catkin_install_python(PROGRAMS scripts/VPI_stereo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/VPI_stereo_CUDA.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/VPI_stereo_PVA.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
