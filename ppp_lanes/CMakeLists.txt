cmake_minimum_required(VERSION 3.10)

project(lanes_detection)

find_package(OpenMP)
find_package(catkin REQUIRED COMPONENTS rosbag )
#find_package(rosbag REQUIRED)

include_directories(${rosbag_INCLUDE_DIRS})
include_directories(.)

add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME} -lboost_system ${catkin_LIBRARIES} OpenMP::OpenMP_CXX)
