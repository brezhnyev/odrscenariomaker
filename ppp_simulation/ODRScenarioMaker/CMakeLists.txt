cmake_minimum_required(VERSION 3.10)

project(ODRScenarioMaker)

set(CMAKE_AUTOMOC ON)
find_package(Qt5 COMPONENTS Core Gui Widgets Xml OpenGL REQUIRED)

include_directories(${Qt5Xml_INCLUDE_DIRS})
include_directories(${Qt5OpenGL_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} main.cpp MainWindow.cpp Viewer.cpp Canvas.cpp Waypath.cpp Waypoint.cpp treeitem.cpp treemodel.cpp treeview.cpp scenario.cpp WaypointProps.cpp Vehicle.cpp)
target_link_libraries(${PROJECT_NAME} libQGLViewer-qt5.so Qt5::Core Qt5::Gui Qt5::Widgets libOpenGL.so Qt5::OpenGL libpthread.so)

add_subdirectory(client)