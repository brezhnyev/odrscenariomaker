# Need to find both Qt4 and QGLViewer if the QQL support is to be built
#FIND_PACKAGE(Qt5 COMPONENTS QtCore QtXml QtOpenGL QtGui)

#include(${QT_USE_FILE})

FIND_PATH(QGLVIEWER_INCLUDE_DIR QGLViewer/qglviewer.h
    ${QGLVIEWER_ROOT}/include
    ENV QGLVIEWERROOT
)


find_library(QGLVIEWER_LIBRARY_RELEASE
  NAMES QGLViewer-qt5
  PATHS ${QGLVIEWER_ROOT}/bin
		/usr/lib
        /usr/local/lib
        /opt/local/lib
        /sw/lib
        ENV QGLVIEWERROOT
        ENV LD_LIBRARY_PATH
        ENV LIBRARY_PATH
)

find_library(QGLVIEWER_LIBRARY_DEBUG
  NAMES QGLViewer-qt5d
  PATHS ${QGLVIEWER_ROOT}/bin
		/usr/lib
        /usr/local/lib
        /opt/local/lib
        /sw/lib
        ENV QGLVIEWERROOT
        ENV LD_LIBRARY_PATH
        ENV LIBRARY_PATH  
)

if(QGLVIEWER_LIBRARY_RELEASE)
  if(QGLVIEWER_LIBRARY_DEBUG)
    set(QGLVIEWER_LIBRARY optimized ${QGLVIEWER_LIBRARY_RELEASE} debug ${QGLVIEWER_LIBRARY_DEBUG})
  else()
    set(QGLVIEWER_LIBRARY ${QGLVIEWER_LIBRARY_RELEASE})
  endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(QGLVIEWER DEFAULT_MSG QGLVIEWER_INCLUDE_DIR QGLVIEWER_LIBRARY)

