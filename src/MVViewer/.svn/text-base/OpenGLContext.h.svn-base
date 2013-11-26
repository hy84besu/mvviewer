/*
 *  OpenGLContext.h
 *  src
 *
 *  Created by Andrei Zaharescu on 22/12/06.
 *  Copyright 2006 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef OPENGL_CONTEXT_H
#define OPENGL_CONTEXT_H

#include <QGLViewer/qglviewer.h>
#include <QMutex>
#ifdef __USE_PBUFFER__
#include "PBuffer.h"
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Class that wraps an OpenGL context - it provides some house-keeping
class OpenGLContext {
public: //TODO: make these private again
	QGLViewer *viewer;
	static QMutex mutex;
	static int mutex_count;
#ifdef __USE_PBUFFER__
	PBuffer* pbuffer;
#endif
	bool manage_context;

	OpenGLContext(QGLViewer*_viewer,bool _manage_context=FALSE);
#ifdef __USE_PBUFFER__
	OpenGLContext(PBuffer* _pbuffer, bool _manage_context=FALSE);
#endif	
	~OpenGLContext();
	
	bool makeCurrent();	
	bool doneCurrent();	
};

#endif