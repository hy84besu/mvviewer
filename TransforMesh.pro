TEMPLATE = lib
VERSION = 0.5.0
TARGET = TransforMesh
QMAKE_MACOSX_DEPLOYMENT_TARGET = 10.6
#DESTDIR = /usr/local/lib
DESTDIR = lib

include(TransforMesh.files)


linux-g++{
	QMAKE_CXXFLAGS += -fPIC
}

