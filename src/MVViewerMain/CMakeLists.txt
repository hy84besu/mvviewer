#sample https://github.com/qknight/springrts.com-random-map-generator/blob/master/src/CMakeLists.txt

cmake_minimum_required(VERSION 2.6)
project(MVViewerMain)

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

# includes
include_directories(
  # locals
  "../Utils/"
  "../TransforMesh/"
  "../MVViewer/"
  "../"
  # the UI files generated by MVViewer project
  "${CMAKE_CURRENT_BINARY_DIR}/../MVViewer/"
)

#target
add_executable (MVViewerMain MACOSX_BUNDLE mvviewer_main.cc) 

#link
target_link_libraries (MVViewerMain TransforMesh MVViewer)
