cmake_minimum_required(VERSION 2.6)
project(MeshMatching)

# find dependencies
find_package(CGAL REQUIRED COMPONENTS Core )

# include dependencies
include( ${CGAL_USE_FILE} )

# defines
ADD_DEFINITIONS( "-DCGAL_USE_F2C -DCGAL_NDEBUG -DCONVEX_SEG -DWITH_CGAL_BLAS_NO -DCGAL_CFG_NO_CPP0X_VARIADIC_TEMPLATES" )

# sources
set(proj_SRC
  MeshMatching.cpp
  FeatureGradientHistogram.cpp
)

# headers
set(proj_HDR
  MeshMatching.h
  FeatureGradientHistogram.h
)

# include
include_directories(
  "../Utils/"
  "../TransforMesh/"
  "../"
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
)

#target
add_executable (MeshMatching ${proj_SRC} ${proj_HDR} mesh_matching_main.cc) 

#link
#link_directories (../TransforMesh) 
target_link_libraries(MeshMatching TransforMesh)
