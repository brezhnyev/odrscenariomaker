cmake_minimum_required(VERSION 3.10)

project(ODRScenarioMaker)

add_compile_options(-fPIC -fopenmp)

#set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-fsanitize=address")

#set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-Wl,--warn-unresolved-symbols")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_AUTOMOC ON)

option(USE_CARLA "Will need carla specific client libs if set to true" ON)

set(CARLA_PATH "/home/carla" CACHE STRING "carla (git) path")

find_package(yaml-cpp)
find_package(Qt5 COMPONENTS Core Gui Widgets Xml OpenGL REQUIRED)

include_directories(${CMAKE_CURRENT_LIST_DIR}/thirdparty/odrparser/include)
include_directories(${CMAKE_CURRENT_LIST_DIR}/thirdparty/odrparser)
include_directories(${CMAKE_CURRENT_LIST_DIR}/thirdparty/tinyxml2/)
include_directories(${CMAKE_CURRENT_LIST_DIR}/thirdparty/yaml-cpp/)
include_directories(${CMAKE_CURRENT_LIST_DIR}/thirdparty/cpp-spline/)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/.)
if (USE_CARLA)
include_directories(${CARLA_PATH}/LibCarla/source)
include_directories(${CARLA_PATH}/Build/rpclib-v2.2.1_c5-c8-libstdcxx-install/include)
include_directories(${CARLA_PATH}/Build/recast-0b13b0-c8-install/include)
include_directories(${CARLA_PATH}/Build/boost-1.72.0-c8-install/include)
endif()

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

set(SOURCE
    main.cpp
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
    CameraProps.cpp
)

if (USE_CARLA)
set(SOURCE ${SOURCE} carlaClient.cpp)
endif()

add_executable(${PROJECT_NAME} ${SOURCE})

set(LIBS
    -lQGLViewer-qt5
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
    -lGLU
    -lXodrBuilder
)

if (USE_CARLA)
set(LIBS ${LIBS}
    ${CARLA_LIB}
    libboost_filesystem.so
    -lrpc
    -lDetour
    -lDetourCrowd
    -lRecast
    -lodrparser
)
endif()

target_link_libraries(${PROJECT_NAME} ${LIBS})
