/*
 *  MVAlgWindow.h
 *  src
 *
 *  Created by Andrei Zaharescu on 15/01/07.
 *  Copyright 2007 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef MV_ALG_VERIF_VIEWER_H
#define MV_ALG_VERIF_VIEWER_H

#include "MVWindow.h"
#include "MVAlgVerif.h"

#include <QFileDialog>

#if QT_VERSION >= 0x040000
#include "ui_MeshMorphingProperties.h"

#define BOOL_TO_STATE(X) ((X)?(Qt::Checked):(Qt::Unchecked))	

class MeshMorphingProperties : public QWidget, public Ui::MeshMorphingProperties
{
	Q_OBJECT
public:
	MVAlgVerif * alg;
	MVViewer* viewer;

	////////////////////////////////////////////////////////////////////////////////////
	MeshMorphingProperties(QWidget *parent, MVViewer* _viewer): QWidget(parent)
	{
		setupUi(this);
		viewer = _viewer;
		alg=NULL;
	}

	////////////////////////////////////////////////////////////////////////////////////	
	void updateGUIParams() {

		doubleSpinInitRemeshes->setValue(alg->alg_init_remeshes);
		doubleSpinInitResize->setValue(alg->alg_init_bbox_scale);
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
		checkAlgThread->setCheckState(BOOL_TO_STATE(false));
	}
	
private slots:

	////////////////////////////////////////////////////////////////////////////////////
	void on_pushAlgStop_pressed() {
		alg->stopAlgPressed = true;
	}

	
	////////////////////////////////////////////////////////////////////////////////////
	void on_pushAlgRunAll_pressed() {

		QByteArray src_mesh_text = lineSrcMesh->text().toAscii();
		char * src_mesh = src_mesh_text.data();		

		QByteArray src_vec_text = lineSrcVecF->text().toAscii();
		char * src_vec = src_vec_text.data();		

		bool reposition = checkReposition->isChecked();
		
		int iter = spinIterations->value();
		float scales = doubleSpinScale->value();
		int init_remeshes = (int)doubleSpinInitRemeshes->value();
		double init_bbox_scale = doubleSpinInitResize->value(); 
		int init_clones = (int) doubleSpinClones->value();		
		double dt = doubleSpinDt->value();
		double smoothing = doubleSpinSmoothing->value();
		bool save_snapshots = checkSaveSnapshots->isChecked();
		bool autoStop = checkAutoStop->isChecked();
		
		QByteArray save_snapshots_prefix_text = lineSaveSnapshotPrefix->text().toAscii();
		char *save_snapshots_prefix = save_snapshots_prefix_text.data();
		
		alg->setParams(src_mesh, src_vec, reposition, init_remeshes, init_bbox_scale, init_clones, dt, iter, scales, smoothing, autoStop, save_snapshots, save_snapshots_prefix);
		if (checkAlgThread->isChecked())
			alg->start();
		else
			alg->execute();
	}

};


class MVAlgVerifWindow : public MVWindow {
	Q_OBJECT
public:
	MeshMorphingProperties *algProps;
	
	////////////////////////////////////////////////////////////////////////////////////
	MVAlgVerifWindow():MVWindow() {
		algProps = new MeshMorphingProperties(tabWidget,viewer);
		tabWidget->addTab(algProps,"Morphing");
		tabWidget->setCurrentWidget(algProps);
		setWindowTitle("Mesh Morphing");
	}
	
	////////////////////////////////////////////////////////////////////////////////////
	void setAlg(MVAlgVerif *_alg) {
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
