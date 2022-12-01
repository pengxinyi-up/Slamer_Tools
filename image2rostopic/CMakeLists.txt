cmake_minimum_required(VERSION 3.0.2)
project(image2rostopic)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

FIND_PACKAGE(OpenCV REQUIRED)
message(STATUS "version: ${OpenCV_VERSION}")

include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(image2rostopic src/image2rostopic.cpp)
#out path:   ~/catkin_ws/devel/lib/rosopencv

target_link_libraries(image2rostopic ${OpenCV_LIBS} ${catkin_LIBRARIES})
