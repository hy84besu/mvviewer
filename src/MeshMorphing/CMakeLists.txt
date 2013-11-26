cmake_minimum_required(VERSION 2.6)
project(MeshMorphing)

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
  MVAlgVerif.cpp
  MVAlgVerifWindow.cpp
)

# moc header files
SET(proj_MOC_HDR
  MVAlgVerif.h
  MVAlgVerifWindow.h
)

# UI files
SET(proj_UI
  MeshMorphingProperties.ui
)

# generate and add the moc files to the list of sources
QT4_WRAP_CPP(proj_SRC ${proj_MOC_HDR} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)

# generate and add the ui files to the list of sources
QT4_WRAP_UI(proj_SRC ${proj_UI})


# include
include_directories(
  "../Utils/"
  "../TransforMesh/"
  "../MVViewer/"
  "../"
  "${CMAKE_CURRENT_BINARY_DIR}/../MVViewer/"
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
)

#target
add_executable (MeshMorphing ${proj_SRC} morphing_main.cc) 
# MACOSX_BUNDLE

#link
#link_directories (../TransforMesh) 
target_link_libraries (MeshMorphing TransforMesh MVViewer)
