cmake_minimum_required(VERSION 3.0.2)
project(offb_sim)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES offb_sim
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME}_node src/offb_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

add_executable(vision_topic_handler src/vision_topic_handler.cpp)
target_link_libraries(vision_topic_handler ${catkin_LIBRARIES})



