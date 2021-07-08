cmake_minimum_required(VERSION 3.10)

project(carla2osi)

add_compile_options("-fPIC")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/modules)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5 COMPONENTS Core Gui Widgets Xml OpenGL REQUIRED)
#find_package(open_simulation_interface REQUIRED)

## OpenGL
find_package(OpenGL REQUIRED)
if(OPENGL_FOUND)
    message(STATUS "OpenGL Found...")
    if(OPENGL_GLU_FOUND)
        message(STATUS "OpenGL GLU Found...")
    endif()
    include_directories(${OPENGL_INCLUDE_DIR})
endif()

## Find QGLViewer
find_package(QGLViewer REQUIRED)
include_directories(${QGLVIEWER_INCLUDE_DIR})
message(STATUS ${QGLVIEWER_INCLUDE_DIR})

set (SOURCE carla2osi.cpp Viewer.cpp Osiexporter.cpp)

include_directories(${CMAKE_CURRENT_LIST_DIR})
include_directories(${Qt5Core_INCLUDE_DIRS})
include_directories(${Qt5Widgets_INCLUDE_DIRS})
include_directories(${Qt5Xml_INCLUDE_DIRS})
include_directories(${Qt5OpenGL_INCLUDE_DIRS})

link_directories("/usr/local/lib/osi3")

add_executable(${PROJECT_NAME} ${SOURCE})
target_link_libraries(${PROJECT_NAME} ${QGLVIEWER_LIBRARY} ${Qt5Core_LIBRARIES} ${OPENGL_LIBRARIES} ${Qt5OpenGL_LIBRARIES} -lopen_simulation_interface -lprotobuf)