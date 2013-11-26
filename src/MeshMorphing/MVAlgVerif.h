/*
 *  MVAlg.h
 *  src
 *
 *  Created by Andrei Zaharescu on 15/12/06.
 *  Copyright 2006 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef MVALGVERIF_H
#define MVALGVERIF_H

#include "OpenGLContext.h"
#include <QThread>
#include "MVData.h"
/////////////////////////////////////////////////////////////////////////////////////////////////////
class MVAlgVerif: public QThread {
Q_OBJECT
public:
	MVAlgVerif(MVData*);
	~MVAlgVerif();

	void setParams(const char* src_mesh_file, const char* src_vec_file_, bool reposition, int init_remeshes_, double init_bbox_scale_, int init_clones_, double dt_, int iters_, double scales_, double smoothing_, bool autoStop_, bool saveOutputs_, char* saveOutput_prefix_);
	void execute() { run();}
	void EndAlg();
signals:
	void stepFinished();	
protected:
	virtual void run();	
	float RunStep();	
	float RunColorStep();	
	void selectClosest_tmp();
	
	// members
public:
	MVData *data;
	Mesh dst_mesh;
	// final alg_params
	bool alg_reposition;	
	int alg_init_remeshes;
	double alg_init_bbox_scale;
	int alg_init_clones;	
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
	
	bool stopAlgPressed;
protected:	
	bool is_init;
	int cur_iter;
};

#endif
