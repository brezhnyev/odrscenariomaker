cmake_minimum_required(VERSION 3.10)

project(ODRScenarioMaker)

#set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-fsanitize=address")

set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-Wl,--warn-unresolved-symbols")
set(CMAKE_AUTOMOC ON)

find_package(yaml-cpp)
find_package(Qt5 COMPONENTS Core Gui Widgets Xml OpenGL REQUIRED)


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/.)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

link_directories(/usr/local/lib/)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

if(CMAKE_BUILD_TYPE MATCHES Debug)
set(CARLA_LIB carla_client_debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")
else()
set(CARLA_LIB carla_client)
endif()

include_directories(${Qt5Xml_INCLUDE_DIRS})
include_directories(${Qt5OpenGL_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} main.cpp 
MainWindow.cpp Viewer.cpp Canvas.cpp Waypath.cpp Waypoint.cpp treeitem.cpp treemodel.cpp treeview.cpp 
scenario.cpp WaypointProps.cpp Vehicle.cpp Selectable.cpp WaypathProps.cpp Actor.cpp ActorProps.cpp IPC.cpp Serializer.cpp ScenarioProps.cpp)

target_link_libraries(${PROJECT_NAME} libQGLViewer-qt5.so Qt5::Core Qt5::Gui Qt5::Widgets libOpenGL.so Qt5::OpenGL libpthread.so yaml-cpp -lGL
${CARLA_LIB}  png jpeg tiff rt pthread rpc Detour DetourCrowd boost_filesystem yaml-cpp)
