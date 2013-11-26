TEMPLATE = app
TARGET = MeshMatching
DEPENDPATH += src/MeshMatching
INCLUDEPATH +=src/MeshMatching

include(TransforMesh.files)

macx {
    CONFIG += app_bundle x86_64
}

# Input
HEADERS += src/MeshMatching/MeshMatching.h \
   	   src/MeshMatching/FeatureGradientHistogram.h

SOURCES += src/MeshMatching/mesh_matching_main.cc \
           src/MeshMatching/MeshMatching.cpp \
   	   src/MeshMatching/FeatureGradientHistogram.cpp

