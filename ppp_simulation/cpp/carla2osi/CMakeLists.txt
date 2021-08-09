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

set (SOURCE Viewer.cpp Osiexporter.cpp thirdparty/odrparser/odr_1_5.cc thirdparty/odrparser/loadFile.cc thirdparty/tinyxml2/tinyxml2.cpp)

include_directories(${CMAKE_CURRENT_LIST_DIR})
include_directories(${CMAKE_CURRENT_LIST_DIR}/thirdparty/odrparser/include)
include_directories(${CMAKE_CURRENT_LIST_DIR}/thirdparty/odrparser)
include_directories(${CMAKE_CURRENT_LIST_DIR}/thirdparty/tinyxml2/)
include_directories(${Qt5Core_INCLUDE_DIRS})
include_directories(${Qt5Widgets_INCLUDE_DIRS})
include_directories(${Qt5Xml_INCLUDE_DIRS})
include_directories(${Qt5OpenGL_INCLUDE_DIRS})
include_directories(/home/kbrezhnyev/tools/carla/Util/OSM2ODR/src/)

link_directories("/usr/local/lib/osi3")
link_directories("~/tools/carla/Util/OSM2ODR/build/src")
link_directories("~/tools/carla/Build/xerces-c-3.2.3-install/lib/")

if(USE_CARLA)
    ADD_DEFINITIONS(-DUSE_CARLA)
endif()

if (USE_CARLA)
    add_executable(${PROJECT_NAME} carla2osi.cpp ${SOURCE})
    target_link_libraries(${PROJECT_NAME} ${CARLA_LIB} png jpeg tiff rt pthread rpc Detour DetourCrowd boost_filesystem ${QGLVIEWER_LIBRARY} ${Qt5Core_LIBRARIES} ${OPENGL_LIBRARIES} ${Qt5OpenGL_LIBRARIES} -lopen_simulation_interface -lprotobuf)

    add_executable(osm2odr osm2odr.cpp)
    target_link_libraries(osm2odr -losm2odr -lxerces-c -lz)
endif()

add_executable(static2osi static2osi.cpp ${SOURCE})
target_link_libraries(static2osi png jpeg tiff rt pthread boost_filesystem ${QGLVIEWER_LIBRARY} ${Qt5Core_LIBRARIES} ${OPENGL_LIBRARIES} ${Qt5OpenGL_LIBRARIES} -lopen_simulation_interface -lprotobuf)