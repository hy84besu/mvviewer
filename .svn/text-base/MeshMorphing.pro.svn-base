TEMPLATE = app
TARGET = MeshMorphing
DEPENDPATH += src/MeshMorphing 
INCLUDEPATH +=src/MeshMorphing

include(TransforMesh.files)
include(MVViewer.files)


macx {
    CONFIG += app_bundle x86
}

# Input
HEADERS += src/MeshMorphing/MVAlgVerif.h \
           src/MeshMorphing/MVAlgVerifWindow.h
FORMS += src/MeshMorphing/MeshMorphingProperties.Qt4.ui
SOURCES += src/morphing_main.cc \
           src/MeshMorphing/MVAlgVerif.cpp \
           src/MeshMorphing/MVAlgVerifWindow.cpp

