cmake_minimum_required(VERSION 3.10)

project(ODRScenarioMaker)

#set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-fsanitize=address")

#set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-Wl,--warn-unresolved-symbols")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_AUTOMOC ON)

set(CARLA_PATH "/home/carla" CACHE STRING "carla (git) path")

find_package(yaml-cpp)
find_package(Qt5 COMPONENTS Core Gui Widgets Xml OpenGL REQUIRED)
find_package(catkin REQUIRED COMPONENTS rosbag )
include_directories("${rosbag_INCLUDE_DIRS}")

include_directories(${CMAKE_CURRENT_LIST_DIR}/thirdparty/odrparser/include)
include_directories(${CMAKE_CURRENT_LIST_DIR}/thirdparty/odrparser)
include_directories(${CMAKE_CURRENT_LIST_DIR}/thirdparty/tinyxml2/)
include_directories(${CMAKE_CURRENT_LIST_DIR}/thirdparty/yaml-cpp/)
include_directories(${CMAKE_CURRENT_LIST_DIR}/thirdparty/cpp-spline/)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/.)
include_directories(${CARLA_PATH}/LibCarla/source)
include_directories(${CARLA_PATH}/Build/rpclib-v2.2.1_c5-c8-libstdcxx-install/include)
include_directories(${CARLA_PATH}/Build/recast-0b13b0-c8-install/include)
include_directories(${CARLA_PATH}/Build/boost-1.72.0-c8-install/include)

link_directories(/usr/local/lib/)
link_directories(${CARLA_PATH}/Build/rpclib-v2.2.1_c5-c8-libstdcxx-install/lib)
link_directories(${CARLA_PATH}/Build/recast-0b13b0-c8-install/lib)
link_directories(${CARLA_PATH}/Build/boost-1.72.0-c8-install/lib)

if(CMAKE_BUILD_TYPE MATCHES Debug)
set(CARLA_LIB carla_client_debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")
link_directories(${CARLA_PATH}/Build/libcarla-client-build.debug/LibCarla/cmake/client)
else()
set(CARLA_LIB carla_client)
link_directories(${CARLA_PATH}/Build/libcarla-client-build.release/LibCarla/cmake/client)
endif()

include_directories(${Qt5Xml_INCLUDE_DIRS})
include_directories(${Qt5OpenGL_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}
    main.cpp
    carlaClient.cpp
    thirdparty/odrparser/odr_1_5.cc
    thirdparty/odrparser/loadFile.cc
    thirdparty/tinyxml2/tinyxml2.cpp
    CanvasXODR.cpp
    thirdparty/cpp-spline/CatmullRom.cpp
    thirdparty/cpp-spline/Vector.cpp
    thirdparty/cpp-spline/Curve.cpp
    thirdparty/cpp-spline/BSpline.cpp
    MainWindow.cpp
    Viewer.cpp
    Canvas.cpp
    Waypath.cpp
    Waypoint.cpp
    treeitem.cpp
    treemodel.cpp
    treeview.cpp
    scenario.cpp
    WaypointProps.cpp
    Vehicle.cpp
    Selectable.cpp
    WaypathProps.cpp
    Actor.cpp
    ActorProps.cpp
    IPC.cpp
    Serializer.cpp
    ScenarioProps.cpp
    World3D.cpp
)

target_link_libraries(${PROJECT_NAME}
    libQGLViewer-qt5.a
    Qt5::Xml
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    libOpenGL.so
    Qt5::OpenGL
    libpthread.so
    yaml-cpp
    -lGL
    png
    jpeg
    tiff
    rt
    pthread
    ${CARLA_LIB}
    ${catkin_LIBRARIES}
    libboost_filesystem.so
    -lrpc
    -lGLU
    -lrpc
    -lDetour
    -lDetourCrowd
    -lRecast
)
