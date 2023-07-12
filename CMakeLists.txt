cmake_minimum_required(VERSION 3.10)

project(ODRScenarioMaker)

add_compile_options(-fPIC -fopenmp)

#set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-fsanitize=address")

#set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-Wl,--warn-unresolved-symbols")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_AUTOMOC ON)

option(USE_CARLA "Will need carla specific client libs if set to true" ON)

if (USE_CARLA)
  add_definitions(-DUSE_CARLA)
endif()

set(CARLA_PATH "/home/carla" CACHE STRING "carla (git) path")

find_package(Qt5 COMPONENTS Core Gui Widgets Xml OpenGL REQUIRED)

include_directories(${CMAKE_CURRENT_LIST_DIR}/src/thirdparty/yaml-cpp/include)
include_directories(${CMAKE_CURRENT_LIST_DIR}/src/thirdparty/cpp-spline/)
include_directories(${CMAKE_CURRENT_LIST_DIR}/src/thirdparty/xodr-builder/)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/.)
if (USE_CARLA)
include_directories(${CARLA_PATH}/LibCarla/source)
include_directories(${CARLA_PATH}/Build/rpclib-v2.2.1_c5-c8-libstdcxx-install/include)
include_directories(${CARLA_PATH}/Build/recast-0b13b0-c8-install/include)
include_directories(${CARLA_PATH}/Build/boost-1.72.0-c8-install/include)
endif()

link_directories(${CMAKE_CURRENT_LIST_DIR}/src/thirdparty/yaml-cpp/lib)
link_directories(/usr/local/lib/)
if (USE_CARLA)
link_directories(${CARLA_PATH}/Build/rpclib-v2.2.1_c5-c8-libstdcxx-install/lib)
link_directories(${CARLA_PATH}/Build/recast-0b13b0-c8-install/lib)
link_directories(${CARLA_PATH}/Build/boost-1.72.0-c8-install/lib)
endif()

if (USE_CARLA)
if(CMAKE_BUILD_TYPE MATCHES Debug)
set(CARLA_LIB carla_client_debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")
link_directories(${CARLA_PATH}/Build/libcarla-client-build.debug/LibCarla/cmake/client)
else()
set(CARLA_LIB carla_client)
link_directories(${CARLA_PATH}/Build/libcarla-client-build.release/LibCarla/cmake/client)
endif()
endif()

include_directories(${Qt5Xml_INCLUDE_DIRS})
include_directories(${Qt5OpenGL_INCLUDE_DIRS})
include_directories(src)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/src/thirdparty/yaml-cpp)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/src/thirdparty/xodr-builder)

set(SOURCE
    src/main.cpp
    src/CanvasXODR.cpp
    src/thirdparty/cpp-spline/CatmullRom.cpp
    src/thirdparty/cpp-spline/Vector.cpp
    src/thirdparty/cpp-spline/Curve.cpp
    src/thirdparty/cpp-spline/BSpline.cpp
    src/MainWindow.cpp
    src/LDoubleSpinBox.cpp
    src/Viewer.cpp
    src/Camera.cpp
    src/Waypath.cpp
    src/Waypoint.cpp
    src/treemodel.cpp
    src/treeview.cpp
    src/scenario.cpp
    src/WaypointProps.cpp
    src/Vehicle.cpp
    src/Walker.cpp
    src/Selectable.cpp
    src/WaypathProps.cpp
    src/Actor.cpp
    src/ActorProps.cpp
    src/Serializer.cpp
    src/ScenarioProps.cpp
    src/World3D.cpp
    src/CameraProps.cpp
)

if (USE_CARLA)
set(SOURCE ${SOURCE} src/carlaClient.cpp)
endif()

add_executable(${PROJECT_NAME} ${SOURCE})

set(LIBS
    -lQGLViewer-qt5
    Qt5::Xml
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    Qt5::OpenGL
    libpthread.so
    yaml-cpp
    -lGL
    png
    jpeg
    tiff
    rt
    pthread
    -lGLU
    XodrBuilder
)

if (USE_CARLA)
set(LIBS ${LIBS}
    ${CARLA_LIB}
    libboost_filesystem.so
    -lrpc
    -lDetour
    -lDetourCrowd
    -lRecast
)
endif()

target_link_libraries(${PROJECT_NAME} ${LIBS})
