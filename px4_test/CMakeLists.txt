cmake_minimum_required(VERSION 3.0.2)
project(px4_test)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf2
  tf2_ros
)

find_package(Eigen3 REQUIRED)




catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES px4_test
#  CATKIN_DEPENDS roscpp std_msgs tf2 tf2_ros
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
include_directories(${EIGEN3_INCLUDE_DIRS})


add_executable(offboard_velocity_example src/offboard_velocity_example.cpp)
add_executable(offboard_position_example src/offboard_position_example.cpp)
add_executable(check_connection src/check_connection.cpp)

target_link_libraries(offboard_velocity_example ${catkin_LIBRARIES})
target_link_libraries(offboard_position_example ${catkin_LIBRARIES})
target_link_libraries(check_connection ${catkin_LIBRARIES})