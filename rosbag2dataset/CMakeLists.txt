cmake_minimum_required(VERSION 2.8.3)
project(rosbag2dataset)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

#set(cv_bridge_34_DIR /home/pxy/Third-Lib/cv_bridge_34/build)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
#   gps_common
  roscpp
  rospy
  rosbag
  pcl_ros
  std_msgs
  image_transport
  cv_bridge_34
  tf
  genmsg
)



find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})


generate_messages(
   DEPENDENCIES
   std_msgs
   nav_msgs   
   geometry_msgs
   sensor_msgs
   tf
 )

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
	${CV_BRIDGE_INCLUDE_DIRS}
)


catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy rosbag std_msgs message_runtime
  DEPENDS EIGEN3 PCL 
  OpenCV
  INCLUDE_DIRS include
)

add_executable(rosbag2dataSet src/bag2dataset.cpp src/read_txt.cpp)
target_link_libraries(rosbag2dataSet ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})




