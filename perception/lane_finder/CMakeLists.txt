cmake_minimum_required(VERSION 2.8.3)
project(lane_finder)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  OpenCV
  image_transport
)

catkin_package()

find_package(OpenCV)
#include_directories(${OpenCV_INCLUDE_DIRS})
#target_link_libraries(my_awesome_library ${OpenCV_LIBRARIES})

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_executable(lanes_finder src/find_lane.cpp)
target_link_libraries(lanes_finder ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_dependencies(lanes_finder beginner_tutorials_generate_messages_cpp)

