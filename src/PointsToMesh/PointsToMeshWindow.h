/*
 *  MVAlgWindow.h
 *  src
 *
 *  Created by Andrei Zaharescu on 15/01/07.
 *  Copyright 2007 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef POINTS_TO_MESH_VIEWER_H
#define POINTS_TO_MESH_VIEWER_H

#include "MVWindow.h"
#include "PointsToMesh.h"

#include <QFileDialog>

#if QT_VERSION >= 0x040000
#include "ui_PointsToMeshProperties.Qt4.h"

#define BOOL_TO_STATE(X) ((X)?(Qt::Checked):(Qt::Unchecked))	

class PointsToMeshProperties : public QWidget, public Ui::PointsToMeshProperties
{
	Q_OBJECT
public:
	PointsToMesh * alg;
	MVViewer* viewer;

	////////////////////////////////////////////////////////////////////////////////////
	PointsToMeshProperties(QWidget *parent, MVViewer* _viewer): QWidget(parent)
	{
		setupUi(this);
		viewer = _viewer;
		alg=NULL;
	}

	////////////////////////////////////////////////////////////////////////////////////	
	void updateGUIParams() {

		doubleSpinInitRemeshes->setValue(alg->alg_init_remeshes);
		doubleSpinInitResize->setValue(alg->alg_init_bbox_scale);
		lineDstPoints->setText(alg->alg_dstPointsFile);
		lineSrcMesh->setText(alg->alg_srcMeshFile);
		lineSrcVecF->setText(alg->alg_srcVecFFile);		
		

		doubleSpinDt->setValue(alg->alg_dt);	
		doubleSpinScale->setValue(alg->alg_scales);		
		doubleSpinSmoothing->setValue(alg->alg_smoothing);	
		spinIterations->setValue(alg->alg_iters);		
		checkSaveSnapshots->setCheckState(BOOL_TO_STATE(alg->alg_saveOutput));	
		lineSaveSnapshotPrefix->setText(alg->alg_saveOutputPrefix);		
		checkReposition->setCheckState(BOOL_TO_STATE(alg->alg_reposition));
		checkAutoStop->setCheckState(BOOL_TO_STATE(alg->alg_autoStop));						

#if defined(__MACOSX__) || defined(__APPLE__)
		checkAlgThread->setCheckState(BOOL_TO_STATE(true));
#endif
	}
	
private slots:

	////////////////////////////////////////////////////////////////////////////////////
	void on_pushAlgStop_pressed() {
		alg->stopAlgPressed = true;
	}

	
	////////////////////////////////////////////////////////////////////////////////////
	void on_pushAlgRunAll_pressed() {

		QByteArray dst_points_text = lineDstPoints->text().toAscii();
		char * dst_points= dst_points_text .data();		


		QByteArray src_mesh_text = lineSrcMesh->text().toAscii();
		char * src_mesh = src_mesh_text.data();		

		QByteArray src_vec_text = lineSrcVecF->text().toAscii();
		char * src_vec = src_vec_text.data();		

		bool reposition = checkReposition->isChecked();
		
		int iter = spinIterations->value();
		float scales = doubleSpinScale->value();
		int init_remeshes = (int)doubleSpinInitRemeshes->value();
		double init_bbox_scale = doubleSpinInitResize->value(); 
		double dt = doubleSpinDt->value();
		double smoothing = doubleSpinSmoothing->value();
		bool save_snapshots = checkSaveSnapshots->isChecked();
		bool autoStop = checkAutoStop->isChecked();
		
		QByteArray save_snapshots_prefix_text = lineSaveSnapshotPrefix->text().toAscii();
		char *save_snapshots_prefix = save_snapshots_prefix_text.data();
		
		alg->setParams(dst_points, src_mesh, src_vec, reposition, init_remeshes, init_bbox_scale, dt, iter, scales, smoothing, autoStop, save_snapshots, save_snapshots_prefix);
		if (checkAlgThread->isChecked())
			alg->start();
		else
			alg->execute();
	}

};


class PointsToMeshWindow : public MVWindow {
	Q_OBJECT
public:
	PointsToMeshProperties *algProps;
	
	////////////////////////////////////////////////////////////////////////////////////
	PointsToMeshWindow ():MVWindow() {
		algProps = new PointsToMeshProperties(tabWidget,viewer);
		tabWidget->addTab(algProps,"Points To Mesh");
		tabWidget->setCurrentWidget(algProps);
		setWindowTitle("Points To Mesh");
	}
	
	////////////////////////////////////////////////////////////////////////////////////
	void setAlg(PointsToMesh *_alg) {
		if (_alg) {
			algProps->alg = _alg;
			QObject::connect(algProps->alg, SIGNAL(stepFinished()),
							 this , SLOT(updateUI()));
			
			algProps->updateGUIParams();
			viewer->setMVData(algProps->alg->data);
		}
	}
	
	private slots:
	
	////////////////////////////////////////////////////////////////////////////////////
	void updateUI() {
		viewer->updateGL();
		QCoreApplication::processEvents();
	}
	
	
};
#endif

#endif
