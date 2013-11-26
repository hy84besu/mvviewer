TEMPLATE = app
TARGET = MVViewer

include(TransforMesh.files)
include(MVViewer.files)

macx{
	CONFIG+= app_bundle x86
}

SOURCES += src/MVViewerMain/mvviewer_main.cc 
