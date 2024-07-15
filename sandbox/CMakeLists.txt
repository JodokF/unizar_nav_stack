cmake_minimum_required(VERSION 3.0.2)
project(sandbox)


find_package(catkin REQUIRED COMPONENTS 
    roscpp  
    std_msgs
    tf2
    tf2_ros 
)


catkin_package(CATKIN_DEPENDS roscpp std_msgs tf2 tf2_ros)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(sandbox src/sandbox.cpp)
target_link_libraries(sandbox ${catkin_LIBRARIES})



