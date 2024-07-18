cmake_minimum_required(VERSION 3.0.2)
project(tracking_cam)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tracking_cam
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



add_executable(move_cam_to_cinempc src/move_cam_to_cinempc.cpp)
target_link_libraries(move_cam_to_cinempc ${catkin_LIBRARIES})# Eigen3::Eigen)
