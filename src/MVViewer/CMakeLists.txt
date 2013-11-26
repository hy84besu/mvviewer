#sample https://github.com/qknight/springrts.com-random-map-generator/blob/master/src/CMakeLists.txt

cmake_minimum_required(VERSION 2.6)
project(MVViewer)

# find dependencies
find_package(CGAL REQUIRED COMPONENTS Core )
find_package(QGLViewer REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Qt4 REQUIRED QtCore QtGui QtXml QtOpenGL)

# include dependencies
include( ${CGAL_USE_FILE} )
include(${QT_USE_FILE})
include(${QGLVIEWER_INCLUDE_DIR})
include(${OPENGL_INCLUDE_DIR})

# defines
ADD_DEFINITIONS( "-DCGAL_USE_F2C -DCGAL_NDEBUG -DCONVEX_SEG -DWITH_CGAL_BLAS_NO -DCGAL_CFG_NO_CPP0X_VARIADIC_TEMPLATES" )

# sources (including files that inherit from Q_OBJECT)
set(proj_SRC
  MVViewer.cpp
  GeoSegmentation.cpp
  CurvatureSegmentation.cpp
  MVData.cpp
  Resample.cpp
  MVViewer.cpp
  OpenGLContext.cpp
)

# moc header files
SET(proj_MOC_HDR
  BoundingBoxProps.h
  MVTransformProps.h
  MVViewer.h
  MVWindow.h
  GeoSegmentation.h
)

# UI files
SET(proj_UI
  MVWindow.ui
  MVTransformProps.ui 
  BoundingBoxProps.ui
)

# generate and add the moc files to the list of sources
QT4_WRAP_CPP(proj_SRC ${proj_MOC_HDR} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)

# generate and add the ui files to the list of sources
QT4_WRAP_UI(proj_SRC ${proj_UI})

# include
include_directories(
  # local
  "../TransforMesh"
  "../Utils/"

  # common files
  ${CMAKE_CURRENT_SOURCE_DIR}

  # QT4_WRAP_UI writes ui_whatever.h here (that is './' and uic is used)
  ${CMAKE_CURRENT_BINARY_DIR}
)

# target
add_library(MVViewer SHARED ${proj_SRC})

# linker
target_link_libraries (MVViewer TransforMesh ${QT_LIBRARIES} ${QGLVIEWER_LIBRARIES} ${OPENGL_LIBRARIES})
