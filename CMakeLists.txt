cmake_minimum_required(VERSION 3.10)

project(ODRScenarioMaker VERSION 1.0)

add_compile_options(-fPIC -fopenmp)

#set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-fsanitize=address")

#set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-Wl,--warn-unresolved-symbols")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_AUTOMOC ON)

add_definitions(-DQT_NO_VERSION_TAGGING)

# libQGLViewer 2.7.2 is still using qt-5, but is based on qmake+make build system
# later versions are based on cmake configuration system, so can be added as cmake subdirectories
set(OGLVV "2.7.2")
execute_process(COMMAND git checkout v${OGLVV} WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/thirdparty/libQGLViewer)
execute_process(COMMAND qmake WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/thirdparty/libQGLViewer/QGLViewer)
execute_process(COMMAND make WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/thirdparty/libQGLViewer/QGLViewer)

option(USE_CARLA "Will need carla specific client libs if set to true" ON)

if (USE_CARLA)
  add_definitions(-DUSE_CARLA)
endif()

set(CARLA_PATH "/home/carla" CACHE STRING "carla (git) path")

find_package(Qt5 COMPONENTS Core Gui Widgets Xml OpenGL REQUIRED)

include_directories(${CMAKE_CURRENT_LIST_DIR}/src/thirdparty/yaml-cpp/include)
include_directories(${CMAKE_CURRENT_LIST_DIR}/src/thirdparty/cpp-spline/)
include_directories(${CMAKE_CURRENT_LIST_DIR}/src/thirdparty/xodr-builder/)
include_directories(${CMAKE_CURRENT_LIST_DIR}/src/thirdparty/libQGLViewer)

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

link_directories(${CMAKE_CURRENT_LIST_DIR}/src/thirdparty/libQGLViewer/QGLViewer)

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
    src/ScenarioProps.cpp
    src/World3D.cpp
    src/CameraProps.cpp
    src/clients/CarlaClient.cpp
)

if (USE_CARLA)
set(SOURCE ${SOURCE})
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

set_target_properties(${PROJECT_NAME} PROPERTIES
INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
string(REPLACE "." ";" OGLVVLIST ${OGLVV})
list(GET OGLVVLIST 0 OGLVMJ)
list(GET OGLVVLIST 1 OGLVMN)
list(GET OGLVVLIST 2 OGLVPT)
install(FILES "${CMAKE_CURRENT_LIST_DIR}/src/thirdparty/libQGLViewer/QGLViewer/libQGLViewer-qt5.so.${OGLVMJ}.${OGLVMN}.${OGLVPT}" DESTINATION lib)
install(FILES "${CMAKE_CURRENT_LIST_DIR}/src/thirdparty/libQGLViewer/QGLViewer/libQGLViewer-qt5.so.${OGLVMJ}.${OGLVMN}" DESTINATION lib)
install(FILES "${CMAKE_CURRENT_LIST_DIR}/src/thirdparty/libQGLViewer/QGLViewer/libQGLViewer-qt5.so.${OGLVMJ}" DESTINATION lib)
install(FILES "${CMAKE_CURRENT_LIST_DIR}/src/thirdparty/libQGLViewer/QGLViewer/libQGLViewer-qt5.so" DESTINATION lib)
if (USE_CARLA)
install(FILES "${CARLA_PATH}/Build/boost-1.72.0-c8-install/lib/libboost_filesystem.so" DESTINATION lib)
install(FILES "${CARLA_PATH}/Build/boost-1.72.0-c8-install/lib/libboost_filesystem.so.1.72.0" DESTINATION lib)
endif()

include(InstallRequiredSystemLibraries)
SET(CPACK_GENERATOR "DEB")
SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "Capgemini Engineering Germany") #required
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/License.txt")
set(CPACK_PACKAGE_VERSION_MAJOR "${PROJECT_VERSION_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${PROJECT_VERSION_MINOR}")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "qt5-default, libeigen3-dev, libjpeg-dev, libpng-dev, libtiff-dev")
include(CPack)