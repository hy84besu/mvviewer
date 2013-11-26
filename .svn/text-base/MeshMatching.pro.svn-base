TEMPLATE = app
TARGET = MeshMatching
DEPENDPATH += src/MeshMatching
INCLUDEPATH +=src/MeshMatching

include(TransforMesh.files)
include(MVViewer.files)


macx {
    CONFIG += app_bundle x86
}

# Input
HEADERS += src/MeshMatching/MeshMatching.h \
           src/MeshMatching/MeshMatchingWindow.h
FORMS += src/MeshMatching/MeshMatchingProperties.Qt4.ui
SOURCES += src/mesh_matching_main.cc \
           src/MeshMatching/MeshMatching.cpp \
           src/MeshMatching/MeshMatchingWindow.cpp

