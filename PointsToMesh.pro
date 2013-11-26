TEMPLATE = app
TARGET = PointsToMesh
DEPENDPATH += src/PointsToMesh
INCLUDEPATH +=src/PointsToMesh
INCLUDEPATH +=src/MeshMatching

include(TransforMesh.files)
include(MVViewer.files)


macx {
    CONFIG += app_bundle x86
}

# Input
HEADERS += src/PointsToMesh/PointsToMesh.h \
           src/PointsToMesh/PointsToMeshWindow.h \
   	   src/MeshMatching/FeatureGradientHistogram.h
FORMS += src/PointsToMesh/PointsToMeshProperties.ui
SOURCES += src/PointsToMesh/points_to_mesh_main.cc \
           src/PointsToMesh/PointsToMesh.cpp \
           src/PointsToMesh/PointsToMeshWindow.cpp \
   	   src/MeshMatching/FeatureGradientHistogram.cpp

