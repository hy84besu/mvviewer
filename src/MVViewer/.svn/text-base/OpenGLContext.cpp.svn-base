/*
 *  OpenGLContext.cpp
 *  src
 *
 *  Created by Andrei Zaharescu on 22/12/06.
 *  Copyright 2006 __MyCompanyName__. All rights reserved.
 *
 */

#include "OpenGLContext.h"
#define USE_MUTEXES=1

#define DEBUG_MUTEX(X) ;
//initialize the static class variables
QMutex OpenGLContext::mutex(QMutex::Recursive);
int OpenGLContext::mutex_count = 0; 

///////////////////////////////////////////////////////////////////////////////////////////////////////////
OpenGLContext::OpenGLContext(QGLViewer*_viewer, bool _manage_context){
	viewer = _viewer;
#ifdef __USE_PBUFFER__	
	pbuffer = NULL;
#endif
	manage_context = _manage_context;
}

OpenGLContext::~OpenGLContext() {
	if (manage_context) {
		if (viewer) delete viewer;
#ifdef __USE_PBUFFER__	
		if (pbuffer) delete pbuffer;
#endif
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef __USE_PBUFFER__	
OpenGLContext::OpenGLContext(PBuffer* _pbuffer, bool _manage_context){
	pbuffer = _pbuffer;
	viewer = NULL;
	manage_context = _manage_context;
}
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////////
bool OpenGLContext::makeCurrent(){
	if (viewer) {
#ifdef USE_MUTEXES==1
		mutex.lock();
		if (mutex_count==0) viewer->makeCurrent();
		mutex_count++;
#else
		viewer->makeCurrent();
#endif
		DEBUG_MUTEX(std::cout << "mutex count v(+) = " << mutex_count << std::endl;)
		return true;
	}
#ifdef __USE_PBUFFER__	
	if (pbuffer) {
#ifdef USE_MUTEXES==1		
		mutex.lock();
		if (mutex_count==0) pbuffer->pbuf->makeCurrent();
		mutex_count++;		
#else
		pbuffer->pbuf->makeCurrent();		
#endif
		DEBUG_MUTEX(std::cout << "mutex count p(+) = " << mutex_count << std::endl;)
		return true;		
	}
#endif
	return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
bool OpenGLContext::doneCurrent(){
	if (viewer) {
#ifdef USE_MUTEXES==1				
		mutex_count--;
		if (mutex_count==0) viewer->doneCurrent();
		mutex.unlock();
#endif
		DEBUG_MUTEX(std::cout << "mutex count v(-) = " << mutex_count << std::endl;)
		return true;
	}
#ifdef __USE_PBUFFER__		
	if (pbuffer) {
#ifdef USE_MUTEXES==1		
		mutex_count--;		
		if (mutex_count==0) pbuffer->pbuf->doneCurrent();
		mutex.unlock();
#endif
		DEBUG_MUTEX(std::cout << "mutex count p(-) = " << mutex_count << std::endl;)
		return true;
	}
#endif
	return false;
}