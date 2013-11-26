/*
 *
 *  Created by Andrei Zaharescu on 15/01/07.
 *  Copyright 2007 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef POINTS_TO_MESH_VIEWER_H
#define POINTS_TO_MESH_VIEWER_H

#include "MVWindow.h"
#include "MeshMatching.h"

#include <QFileDialog>

#if QT_VERSION >= 0x040000
#include "ui_MeshMatchingProperties.Qt4.h"

#define BOOL_TO_STATE(X) ((X)?(Qt::Checked):(Qt::Unchecked))	

class MeshMatchingProperties : public QWidget, public Ui::MeshMatchingProperties
{
	Q_OBJECT
public:
	MeshMatching * alg;
	MVViewer* viewer;

	////////////////////////////////////////////////////////////////////////////////////
	MeshMatchingProperties(QWidget *parent, MVViewer* _viewer): QWidget(parent)
	{
		setupUi(this);
		viewer = _viewer;
		alg=NULL;
	}

	////////////////////////////////////////////////////////////////////////////////////	
	void updateGUIParams() {

		lineSrcMesh->setText(alg->alg_srcMeshFile);
		lineDstMesh->setText(alg->alg_dstMeshFile);
		lineSaveOutputFile->setText(alg->alg_saveOutputFile);

		doubleSpinNoRings->setValue(alg->alg_featNoRings);	
		doubleSpinNoBinsCentroid->setValue(alg->alg_featNoBinsCentroid);
		doubleSpinNoBinsGroups->setValue(alg->alg_featNoBinsGroups);
		doubleSpinNoBinsOrientations->setValue(alg->alg_featNoBinsOrientations);
		doubleSpinSpatialInfluence->setValue(alg->alg_featSpatialInfluence);		
		doubleSpinBestMatchesRatio->setValue(alg->alg_matchingBestMatchesRatio);
		checkSaveOutput->setCheckState(BOOL_TO_STATE(alg->alg_saveOutput));
		checkFeaturesOnly->setCheckState(BOOL_TO_STATE(alg->alg_featuresOnly));		
		
		if (strcmp(alg->alg_saveOutputFormat,"sum")==0) comboBoxSaveFormat->setCurrentIndex(0);
		else comboBoxSaveFormat->setCurrentIndex(1);

//		lineSaveSnapshotPrefix->setText(alg->alg_saveOutputPrefix);		

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

		QByteArray src_mesh_text = lineSrcMesh->text().toAscii();
		char * src_mesh = src_mesh_text.data();		

		QByteArray dst_mesh_text = lineDstMesh->text().toAscii();
		char * dst_mesh = dst_mesh_text.data();

		QByteArray save_output_file_text = lineSaveOutputFile->text().toAscii();
		char * save_output_file = save_output_file_text .data();
		
		char * save_output_format;
		if (comboBoxSaveFormat->currentIndex()==0)
			save_output_format = "sum";
		else
			save_output_format = "det";

		int noRings = (int)doubleSpinNoRings->value();
		int noBinsCentroid = (int) doubleSpinNoBinsCentroid->value();
		int noBinsGroup = (int) doubleSpinNoBinsGroups->value();
		int noBinsOrientations = (int) doubleSpinNoBinsOrientations->value();
		double spinSpatialInfluence = doubleSpinSpatialInfluence->value();
		double matchingBestMatchesRatio = doubleSpinBestMatchesRatio->value();
		bool save_output = checkSaveOutput->isChecked();
		bool features_only = checkFeaturesOnly->isChecked();
		alg->setParams(src_mesh, dst_mesh, noRings, true, noBinsCentroid, noBinsGroup , noBinsOrientations , spinSpatialInfluence, matchingBestMatchesRatio, save_output, save_output_file, save_output_format, alg->alg_groundtruthFile, alg->alg_nonMaxSup, alg->alg_scaleSpace, alg->alg_cornerThresh, alg->alg_featType, alg->alg_detType, alg->alg_detThresh, alg->alg_srcMeshDescFile, alg->alg_dstMeshDescFile, features_only,false,0.1,0.1);




/*
		doubleSpinNoRings->setValue(alg->alg_featNoRings);	
		doubleSpinNoBinsCentroid->setValue(alg->alg_featNoBinsCentroid);
		doubleSpinNoBinsGroups->setValue(alg->alg_featNoBinsGroups);
		doubleSpinNoBinsOrientations->setValue(alg->alg_featNoBinsOrientations);
		doubleSpinSpatialInfluence->setValue(alg->alg_featSpatialInfluence);	
		
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
		
		
*/
		if (checkAlgThread->isChecked())
			alg->start();
		else
			alg->execute();
	}

};


class MeshMatchingWindow : public MVWindow {
	Q_OBJECT
public:
	MeshMatchingProperties *algProps;
	
	////////////////////////////////////////////////////////////////////////////////////
	MeshMatchingWindow ():MVWindow() {
		algProps = new MeshMatchingProperties(tabWidget,viewer);
		tabWidget->addTab(algProps,"Mesh Matching");
		tabWidget->setCurrentWidget(algProps);
		setWindowTitle("Mesh Matching");
	}
	
	////////////////////////////////////////////////////////////////////////////////////
	void setAlg(MeshMatching *_alg) {
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
