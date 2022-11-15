cmake_minimum_required(VERSION 3.10)

project(ODR2Poly LANGUAGES C CXX)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/modules)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-fPIC -fopenmp)

set(CMAKE_AUTOMOC ON)

find_package(Qt5 COMPONENTS Core Gui Widgets Xml OpenGL REQUIRED)
find_package(QGLViewer REQUIRED)

include_directories(${CMAKE_SOURCE_DIR})

include_directories(${Qt5Core_INCLUDE_DIRS})
include_directories(${Qt5Widgets_INCLUDE_DIRS})
include_directories(${Qt5Xml_INCLUDE_DIRS})
include_directories(${Qt5OpenGL_INCLUDE_DIRS})
link_directories("/usr/local/lib")

add_executable(${PROJECT_NAME} main.cpp Viewer.cpp CanvasXODR.cpp)

target_link_libraries(
  ${PROJECT_NAME} -lQGLViewer-qt5 -lOpenGL -lGLU Qt5::Core Qt5::Gui Qt5::Xml Qt5::Widgets
  ${QGLVIEWER_LIBRARY} -fopenmp -lodrparser -lXodrBuilder)
