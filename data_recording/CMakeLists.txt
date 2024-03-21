cmake_minimum_required(VERSION 3.0.2)
project(data_recording)

set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS 
    roscpp  
    std_msgs 
)

catkin_package(CATKIN_DEPENDS roscpp std_msgs)


include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(trajectory_recorder_node src/trajectory_recorder.cpp)
target_link_libraries(trajectory_recorder_node ${catkin_LIBRARIES})


