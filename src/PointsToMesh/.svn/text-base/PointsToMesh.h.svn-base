/*
 *  MVAlg.h
 *  src
 *
 *  Created by Andrei Zaharescu on 15/12/06.
 *  Copyright 2006 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef POINTS_TO_MESH_H
#define POINTS_TO_MESH_H

#include "OpenGLContext.h"
#include "PointSet.h"
#include <QThread>
#include "MVData.h"
/////////////////////////////////////////////////////////////////////////////////////////////////////
class PointsToMesh: public QThread {
Q_OBJECT
public:
	PointsToMesh(MVData*);
	~PointsToMesh();

	void setParams(const char* dst_points_file, const char* src_mesh_file, const char* src_vec_file_, bool reposition, int init_remeshes_, double init_bbox_scale_, double dt_, int iters_, double scales_, double smoothing_, bool autoStop_, bool saveOutputs_, char* saveOutput_prefix_);
	void execute() { run();}
	void EndAlg();
signals:
	void stepFinished();	
protected:
	virtual void run();	
	float RunStep();	
	
	// members
public:
	MVData *data;
	// final alg_params
	bool alg_reposition;	
	int alg_init_remeshes;
	double alg_init_bbox_scale;
	double alg_dt;
	double alg_max_step;	
	int alg_scales;	
	int alg_iters;
	double alg_smoothing;
	bool alg_autoStop;
	bool alg_saveOutput;
	bool alg_keepVerticesConstant;
	char alg_saveOutputPrefix[255];
	char alg_srcMeshFile[255];
	char alg_srcVecFFile[255];	
	char alg_dstPointsFile[255];
	bool stopAlgPressed;
protected:	
	bool is_init;
	int cur_iter;

	PointSet* dest_points;
};

#endif
