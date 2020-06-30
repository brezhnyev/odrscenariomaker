cmake_minimum_required(VERSION 3.10)

project(client)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-Wl,--warn-unresolved-symbols")
set(CMAKE_AUTOMOC ON)

find_package(Qt5 COMPONENTS Core Gui Widgets Xml OpenGL REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/.)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/boost)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/carla/)

set (SOURCE play.cpp ../Selectable.cpp ../Waypath.cpp ../Waypoint.cpp)

link_directories(/usr/local/lib/)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/carla_debug)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/carla_release)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/boost)

add_executable(${PROJECT_NAME} ${SOURCE})

if(CMAKE_BUILD_TYPE MATCHES Debug)
set(CARLA_LIB carla_client_debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")
else()
set(CARLA_LIB carla_client)
endif()

message("CARLA_LIB: ${CARLA_LIB}")
target_link_libraries(${PROJECT_NAME} ${CARLA_LIB} 
    png jpeg tiff rt pthread rpc Detour DetourCrowd boost_filesystem)
