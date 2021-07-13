cmake_minimum_required (VERSION 3.10)

project (ppp_carla_client)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS FALSE)

#set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-Wl,--warn-unresolved-symbols")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/.)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/boost)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/carla/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/rpc/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/recast/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/scenarios)

link_directories(/usr/local/lib/)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/carla_debug)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/carla_release)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/boost)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/rpc)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/recast)

if(CMAKE_BUILD_TYPE MATCHES Debug)
set(CARLA_LIB carla_client_debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")
else()
set(CARLA_LIB carla_client)
endif()

# set(SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/ppp_sim_start.cpp ${CMAKE_CURRENT_SOURCE_DIR}/PPPScene.cpp)
# add_executable(${PROJECT_NAME} ${SOURCE})
# message("CARLA_LIB: ${CARLA_LIB}")
# target_link_libraries(${PROJECT_NAME} ${CARLA_LIB} png jpeg tiff rt pthread rpc Detour DetourCrowd boost_filesystem yaml-cpp)

# add_executable(originalExample ./originalExample.cpp)
# target_link_libraries(originalExample ${CARLA_LIB} png jpeg tiff rt pthread rpc Detour DetourCrowd boost_filesystem)

# add_executable(test_spawn ./tests//test_spawn.cpp)
# target_link_libraries(test_spawn ${CARLA_LIB} png jpeg tiff rt pthread rpc Detour DetourCrowd boost_filesystem)

# add_executable(test_osi ./tests/test_osi.cpp)
# target_link_libraries(test_osi ${CARLA_LIB} png jpeg tiff rt pthread rpc Detour DetourCrowd boost_filesystem)

# add_executable(test_left_turns ./tests/test_left_turns.cpp)
# target_link_libraries(test_left_turns ${CARLA_LIB} png jpeg tiff rt pthread rpc Detour DetourCrowd boost_filesystem)

# add_executable(test_alt_turns ./tests/test_alt_turns.cpp)
# target_link_libraries(test_alt_turns ${CARLA_LIB} png jpeg tiff rt pthread rpc Detour DetourCrowd boost_filesystem)

# add_executable(test_walker ./tests/test_walker.cpp)
# target_link_libraries(test_walker ${CARLA_LIB} png jpeg tiff rt pthread rpc Detour DetourCrowd boost_filesystem)

add_executable(scenario_01 ./scenarios/scenario_01.cpp ./scenarios/pwalker.cpp ./scenarios/pvehicle.cpp ./scenarios/ptrafficmanager.cpp)
target_link_libraries(scenario_01 ${CARLA_LIB} png jpeg tiff rt pthread rpc Detour DetourCrowd boost_filesystem yaml-cpp)
