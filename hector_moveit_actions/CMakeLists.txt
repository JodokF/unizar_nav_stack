cmake_minimum_required(VERSION 2.8.3)
set (CMAKE_CXX_STANDARD 11)
project(hector_moveit_actions)

find_package(catkin REQUIRED COMPONENTS 
              roscpp 
              actionlib 
              actionlib_msgs 
              geometry_msgs 
              hector_uav_msgs 
              tf 
              tf2_sensor_msgs  
              tf2_ros
              moveit_core
              moveit_ros_planning_interface
              std_msgs 
              )
find_package(Boost REQUIRED COMPONENTS system)
find_package(octomap REQUIRED)

add_action_files(
  DIRECTORY action
  FILES ExecuteDroneTrajectory.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs geometry_msgs
)
catkin_package(
    CATKIN_DEPENDS roscpp actionlib_msgs hector_uav_msgs moveit_core moveit_ros_planning_interface std_msgs
)


include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(trajectory_controller_dwa src/trajectory_executer_dwa.cpp)
target_link_libraries(trajectory_controller_dwa ${catkin_LIBRARIES})
add_dependencies(
  trajectory_controller_dwa
  ${hector_moveit_actions_EXPORTED_TARGETS} hector_uav_msgs_generate_messages
)

include_directories(include ${OCTOMAP_INCLUDE_DIRS})
add_executable(trajectory_controller_dwa_voxmap src/trajectory_executer_dwa_voxmap.cpp)
target_link_libraries(trajectory_controller_dwa_voxmap ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})
add_dependencies(
  trajectory_controller_dwa_voxmap
  ${hector_moveit_actions_EXPORTED_TARGETS} hector_uav_msgs_generate_messages
)

include_directories(include ${OCTOMAP_INCLUDE_DIRS})
add_executable(tyrion_trayectory_executer_dwa src/tyrion_trayectory_executer_dwa.cpp)
target_link_libraries(tyrion_trayectory_executer_dwa ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})
add_dependencies(
  tyrion_trayectory_executer_dwa
  ${hector_moveit_actions_EXPORTED_TARGETS} hector_uav_msgs_generate_messages
)


add_executable(velocity_command 
    src/velocity_command.cpp
)
target_link_libraries(velocity_command
    ${catkin_LIBRARIES}
)
