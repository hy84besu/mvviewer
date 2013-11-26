cmake_minimum_required(VERSION 2.6)

project(ZaharescuUtils)
SET(CGAL_DIR /opt/local/lib/cmake/)

#SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/opt/local/lib/cmake/")
#SET(CMAKE_MODULE_PATH /opt/local/share/cmake-2.8/Modules)
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules)

#platform
set(CMAKE_OSX_ARCHITECTURES "x86_64")
#export CC=/usr/bin/clang
#export CXX=/usr/bin/clang++

#set(CMAKE_CXX_COMPILER "/usr/bin/g++")
#set(CMAKE_CXX_COMPILER "/usr/bin/clang++")

ADD_SUBDIRECTORY( src ) 

