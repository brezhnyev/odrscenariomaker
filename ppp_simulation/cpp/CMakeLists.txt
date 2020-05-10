cmake_minimum_required (VERSION 3.10)

project (ppp_carla_client)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-Wl,--unresolved-symbols=ignore-all")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/boost)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/carla/)

link_directories(/usr/local/lib/)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/carla)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/boost)

add_executable(${PROJECT_NAME} ../ppp_sim_start.cpp)

if(CMAKE_BUILD_TYPE MATCHES DEBUG)
set(CLARA_LIB carla_client_debug)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} " -g -O0")
else()
set(CLARA_LIB carla_client)
endif()

target_link_libraries(${PROJECT_NAME} ${CLARA_LIB} png jpeg tiff rt pthread rpc Detour DetourCrowd boost_filesystem)