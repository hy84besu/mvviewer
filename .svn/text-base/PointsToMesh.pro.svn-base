TEMPLATE = app
TARGET = PointsToMesh
DEPENDPATH += src/PointsToMesh
INCLUDEPATH +=src/PointsToMesh

include(TransforMesh.files)
include(MVViewer.files)


macx {
    CONFIG += app_bundle x86
}

# Input
HEADERS += src/PointsToMesh/PointsToMesh.h \
           src/PointsToMesh/PointsToMeshWindow.h
FORMS += src/PointsToMesh/PointsToMeshProperties.Qt4.ui
SOURCES += src/points_to_mesh_main.cc \
           src/PointsToMesh/PointsToMesh.cpp \
           src/PointsToMesh/PointsToMeshWindow.cpp

