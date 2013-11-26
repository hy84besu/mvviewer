/*
 *  MVAlgWindow.h
 *  src
 *
 *  Created by Andrei Zaharescu on 15/01/07.
 *  Copyright 2007 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef MV_WINDOW_H
#define MV_WINDOW_H

#include "MVViewer.h"
#include "CurvatureSegmentation.h"
#include "GeoIntegral.h"
#include "GeoSegmentation.h"
#include "MVTransformProps.h"
#include "BoundingBoxProps.h"
#include "PointSet.h"
#include "MeshError.h"
#include "FeatureGradientHistogram.h"

#include <QFileDialog>
#include <QInputDialog>
#include <QMainWindow>
#include <QTimerEvent>

#include <CGAL/Timer.h>
#include <unistd.h>

#if QT_VERSION >= 0x040000
#include "ui_MVWindow.Qt4.h"


class MVWindow : public QMainWindow, public Ui::MVWindow
{
	Q_OBJECT
public:
	GeoIntegral ginteg;
	GeoSegmentation *gseg;
	int InteractiveTimerId;
	////////////////////////////////////////////////////////////////////////////////////
	MVWindow() {
		gseg = new GeoSegmentation();
		setupUi(this); 
		connect(actionLoad_mesh, SIGNAL(triggered()), this, SLOT(actionLoad_mesh_triggered()));
		connect(actionReplace_mesh_with, SIGNAL(triggered()), this, SLOT(actionReplace_mesh_with_triggered()));
		connect(actionSave_mesh, SIGNAL(triggered()), this, SLOT(actionSave_mesh_triggered()));
		connect(actionLoad_vectorfield, SIGNAL(triggered()), this, SLOT(actionLoad_vectorfield_triggered()));
		connect(actionLoad_as_merged_triangles, SIGNAL(triggered()), this, SLOT(actionLoad_as_merged_triangles_triggered()));
	
		connect(actionSave_vectorfield, SIGNAL(triggered()), this, SLOT(actionSave_vectorfield_triggered()));
		connect(actionSave_snapshot, SIGNAL(triggered()), this, SLOT(actionSave_snapshot_triggered()));
		connect(actionConnected_components, SIGNAL(triggered()), this, SLOT(actionConnected_components_triggered()));		
		connect(actionKeep_largest_component, SIGNAL(triggered()), this, SLOT(actionKeep_largest_component_triggered()));				
		connect(actionInfo, SIGNAL(triggered()), this, SLOT(actionInfo_triggered()));				
		connect(actionCompute_intersections, SIGNAL(triggered()), this, SLOT(actionCompute_intersections_triggered()));
		connect(actionMerge_triangle_soup, SIGNAL(triggered()), this, SLOT(actionMerge_triangle_soup_triggered()));
		connect(actionRemove_self_intersections, SIGNAL(triggered()), this, SLOT(actionRemove_self_intersections_triggered()));
		connect(actionShow_TransforMesh_interactively, SIGNAL(triggered()), this, SLOT(actionShow_TransforMesh_interactively_triggered()));		
		connect(actionEnsure_edge_sizes, SIGNAL(triggered()), this, SLOT(actionEnsure_edge_sizes_triggered()));
		connect(actionRemove_Degenerate_Triangles, SIGNAL(triggered()), this, SLOT(actionRemove_Degenerate_Triangles_triggered()));
		
		connect(actionRemove_Invisible_Facets, SIGNAL(triggered()), this, SLOT(actionRemove_Invisible_Facets_triggered()));
		connect(actionRestrict_to_Bounding_Box, SIGNAL(triggered()), this, SLOT(actionRestrict_to_Bounding_Box_triggered()));
		
		connect(actionScale_world, SIGNAL(triggered()), this, SLOT(actionScale_world_triggered()));
		connect(actionCameraInfo, SIGNAL(triggered()), this, SLOT(actionCameraInfo_triggered()));		
		connect(actionGenerate_Random_Colors, SIGNAL(triggered()), this, SLOT(actionGenerate_Random_Colors_triggered()));
		connect(actionBlur_Input_Images, SIGNAL(triggered()), this, SLOT(actionBlur_Input_Images_triggered()));
		connect(actionError_wrt_groundruth_points, SIGNAL(triggered()), this, SLOT(actionError_wrt_groundruth_points_triggered()));

		connect(actionSqrt3_Subdivision, SIGNAL(triggered()), this, SLOT(actionRemesh_Sqrt3_triggered()));
		connect(actionLoop_Subdivision, SIGNAL(triggered()), this, SLOT(actionRemesh_Loop_triggered()));
		connect(actionCatmull_Clark_Subdivision, SIGNAL(triggered()), this, SLOT(actionRemesh_Catmull_Clark_triggered()));
		connect(actionDoo_Sabin_Subdivision, SIGNAL(triggered()), this, SLOT(actionRemesh_Doo_Sabin_triggered()));		

		connect(actionConvolve, SIGNAL(triggered()), this, SLOT(actionConvolve_triggered()));		
		connect(actionBatchConvolve, SIGNAL(triggered()), this, SLOT(actionBatchConvolve_triggered()));				
		
		connect(actionSmooth, SIGNAL(triggered()), this, SLOT(on_pushSmooth_pressed()));				
		
		connect(actionSIFT_Features, SIGNAL(triggered()), this, SLOT(actionSIFT_Features_triggered()));		
		
		connect(gseg, SIGNAL(stepFinished()), this, SLOT(updateUI()));

		connect(actionQuit, SIGNAL(triggered()), this, SLOT(close()) );
		
		//		connect(actionInvert, SIGNAL(triggered()), this, SLOT(actionInvert_triggered()));
//		connect(actionShift, SIGNAL(triggered()), this, SLOT(actionShift_triggered()));
//		connect(actionUnion, SIGNAL(triggered()), this, SLOT(actionUnion_triggered()));				
		
		
		InteractiveTimerId=-1;
	}
	
	
private slots:

	void updateUI()
	{
		viewer->updateGL();
		QCoreApplication::processEvents();
	}

	double getScalingFactor() {
		bool ok;
		return pow(10,doubleScaleFactor->value());
	}


	////////////////////////////////////////////////////////////////////////////////////
	void actionLoad_mesh_triggered() {
		QString fileName = QFileDialog::getOpenFileName(this, tr("Load Mesh"),".", tr("OFF Files - Simple (*.off);; OFF Files - with Normals (*.noff);; OFF Files - with Colors (*.coff);; PLY Files (*.ply)"));
		if (fileName != "") {
			viewer->data->mesh.loadFormat((char*)fileName.toAscii().data()  ,"off");
			//			viewer->data->visual_hull = viewer->data->mesh.p;
			viewer->initSceneRadius();
			viewer->updateGL();
		}
	}

	////////////////////////////////////////////////////////////////////////////////////
	void actionLoad_as_merged_triangles_triggered() {
		QString fileName = QFileDialog::getOpenFileName(this, tr("Load Mesh"),".", tr("OFF Files - Simple (*.off);;"));
		if (fileName != "") {
			viewer->data->mesh.loadAsTriangleSoup((char*)fileName.toAscii().data());
			viewer->initSceneRadius();
			viewer->updateGL();
		}
	}



	////////////////////////////////////////////////////////////////////////////////////
	void actionReplace_mesh_with_triggered() {
		QString fileName = QFileDialog::getOpenFileName(this, tr("Load Mesh"),".", tr("OFF Files - Simple (*.off);; OFF Files - with Normals (*.noff);; OFF Files - with Colors (*.coff);; PLY Files (*.ply)"));
		if (fileName == "") return;
		Mesh tmpMesh;
		bool ok;
		double relScale = QInputDialog::getDouble(this, tr("QInputDialog::getDouble()"),tr("Relative Scale:"), 1.0, -10000, 10000, 2, &ok);
     		if (!ok) return;
		tmpMesh.loadFormat((char*)fileName.toAscii().data()  ,"off");
		viewer->data->mesh.replaceWith(tmpMesh.p,(float)relScale);
		viewer->initSceneRadius();
		viewer->updateGL();
	}


	////////////////////////////////////////////////////////////////////////////////////
	void actionSave_mesh_triggered() {
		QString fileName = QFileDialog::getSaveFileName(this, tr("Save Mesh"), ".", tr("OFF Files - Simple (*.off);; OFF Files - with Normals (*.noff);; OFF Files - with Colors (*.coff);; P3D Files - Vertices with Normals(*.P3D);;PLY Files(*.ply)"));
		if (fileName != "")
			viewer->data->mesh.saveFormat((char*)fileName.toAscii().data());
	}
	
	////////////////////////////////////////////////////////////////////////////////////
	void actionLoad_vectorfield_triggered() {
		QString fileName = QFileDialog::getOpenFileName(this, tr("Load Vectorfield"),".", tr("Diff Files (*.diff)"));
		if (fileName != "") {
			cout << "Loading vector field..";
			cout.flush();		
			viewer->data->mesh.loadVectorField((char*)fileName.toAscii().data(),true);
			cout << "OK"<<endl;					
		}
		
	}
	
	////////////////////////////////////////////////////////////////////////////////////
	void actionSave_vectorfield_triggered() {
		QString fileName = QFileDialog::getSaveFileName(this, tr("Save Vectorfield"),
														".", tr("Diff Files (*.diff)"));
		if (fileName != "") {
			cout << "Saving vector field..";
			cout.flush();		
			viewer->data->mesh.saveVectorField((char*)fileName.toAscii().data());
			viewer->data->mesh.saveVertexIndices("tmp.idx");
			cout << "OK"<<endl;					
		}
		
	}
	
	////////////////////////////////////////////////////////////////////////////////////
	void actionSave_snapshot_triggered() {
		
		if(viewer->openSnapshotFormatDialog()) {
			QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"),".", tr("All Files (*.*)"));
			if (fileName != "") viewer->saveSnap(fileName);
				
			
		}
	}

	
	////////////////////////////////////////////////////////////////////////////////////
	void actionBatchConvolve_triggered() {
		bool ok;
		double noTimes = QInputDialog::getInteger(this, tr("QInputDialog::getInteger()"),tr("Iterations:"), 50, 0, 10000, 2, &ok);
		if (!ok) return;
		char *filenameTemplate ="./%s-measure-%04d.png";
		char filename[255];
		for(int i=0;i<noTimes;i++) {
			cout << "Iteration " << i << "/" << noTimes << endl;
			viewer->setGeometryMode('Q');
			sprintf(filename,filenameTemplate,"qual",i);
			viewer->saveSnap(filename);
			viewer->setGeometryMode('q');
			sprintf(filename,filenameTemplate,"deriv-qual",i);
			viewer->saveSnap(filename);
			viewer->data->mesh.convolve();
		}
		
	}
	
	// Mesh related operations

	////////////////////////////////////////////////////////////////////////////////////	
	void actionConnected_components_triggered() {
		
		viewer->data->mesh.computeConnectedComponents(true);
		viewer->setGeometryMode('C');		
		viewer->data->mesh.updateMeshData();		
		viewer->updateGL();		
	}
	////////////////////////////////////////////////////////////////////////////////////	
	void actionRemove_Invisible_Facets_triggered() {
		viewer->data->mesh.removeInvisibleFacets();
	}
	
	////////////////////////////////////////////////////////////////////////////////////	
	void actionKeep_largest_component_triggered() {
		viewer->data->mesh.keepLargestConnectedComponent();
		viewer->updateGL();
		
	}	

	////////////////////////////////////////////////////////////////////////////////////	
	void actionInfo_triggered() {
		
		viewer->data->mesh.displayInfo();
		for(int i=1;i<4;i++) 
			cout << "Target edge for " << i << " pixels is " << viewer->data->computeDesiredEdgeSize(0,i) <<endl;
	}

	////////////////////////////////////////////////////////////////////////////////////	
	void actionCameraInfo_triggered() {
		 cout << "-------------- Cameras info ------------------------"<<endl;
		 for(int i=0;i<viewer->data->ncameras;i++) {
		 cout << "Camera " << i << endl;
		 viewer->data->cameras[i].display();
		 cout << "----------------------------------------------------"<<endl;
		 }
//		cout << " - current_scale_desired_edge_size = " << viewer->alg->data->computeDesiredEdgeSize(spinScale->value(),targetPixelSize) << endl;
		
	}
	
	////////////////////////////////////////////////////////////////////////////////////
	void actionCompute_intersections_triggered() {
		viewer->data->mesh.checkIntersections();
		viewer->data->mesh.updateMeshData();
		viewer->updateGL();
	}

	////////////////////////////////////////////////////////////////////////////////////
	void actionRemove_self_intersections_triggered() {
		viewer->data->mesh.removeSelfIntersections();
		viewer->updateGL();
	}

	void timerEvent(QTimerEvent *event) {
		if (InteractiveTimerId==event->timerId()) {
			viewer->data->mesh.interactive_colors_max_facet_no++;
			viewer->updateGL();
			if (viewer->data->mesh.p.size_of_facets()<viewer->data->mesh.interactive_colors_max_facet_no)
				killTimer(InteractiveTimerId);
		}

	}
	
	void actionShow_TransforMesh_interactively_triggered() {
		if (InteractiveTimerId>0)
			killTimer(InteractiveTimerId);
		viewer->data->mesh.interactive_colors_max_facet_no=0;
		viewer->data->mesh.removeSelfIntersections();
		viewer->setGeometryMode('R');		
		
		InteractiveTimerId=startTimer(50);
			
		viewer->updateGL();					
		
	}
	
	void actionMerge_triangle_soup_triggered() {
		viewer->data->mesh.mergeTriangleSoup();
		viewer->setGeometryMode('Q');
		viewer->updateGL();
	}
	////////////////////////////////////////////////////////////////////////////////////	
	void actionEnsure_edge_sizes_triggered() {
		bool ok;
		double targetEdge = QInputDialog::getDouble(this, tr("QInputDialog::getDouble()"),
										   tr("Amount:"), viewer->data->mesh.edge_avg, -10000, 10000, 5, &ok);
		if (ok) {
			float minEdgeSize= 0.74*targetEdge;
			float maxEdgeSize = 1.51*targetEdge;
			viewer->data->mesh.ensureEdgeSizes(minEdgeSize,maxEdgeSize,0.2,150);
		}
	}
	
	void actionRemove_Degenerate_Triangles_triggered() {
		viewer->data->mesh.fixDegeneracy(true);
		viewer->updateGL();
	}
	
	
	void actionRemesh_Sqrt3_triggered() {
		viewer->data->mesh.remesh(Mesh::Remesh_Sqrt3);
		viewer->updateGL();	
	}

	void actionRemesh_Loop_triggered() {
		viewer->data->mesh.remesh(Mesh::Remesh_Loop);
		viewer->updateGL();	
	}

	
	void actionRemesh_Catmull_Clark_triggered() {
		viewer->data->mesh.remesh(Mesh::Remesh_CatmullClark);
		viewer->updateGL();	
	}

	void actionRemesh_Doo_Sabin_triggered() {
		viewer->data->mesh.remesh(Mesh::Remesh_DooSabin);
		viewer->updateGL();	
	}
	
	void actionConvolve_triggered() {
		cout << "Convolving mesh with a gaussian kernel" << endl;
		viewer->data->mesh.convolve();
		viewer->updateGL();
	}
	

	void actionSIFT_Features_triggered() {
		Vertex_iterator vi = viewer->data->mesh.p.vertices_begin();
		for(int i=0;i<141;i++) vi++;
		int no_rings=5;
		int no_bins_centroid=36;
		int no_bins_groups=3;
		int no_bins_orientations=8;
		double spatial_influence=0.5;
		FeatureGradientHistogram feature(0,no_rings, no_bins_centroid, no_bins_groups, no_bins_orientations, spatial_influence);
		Vertex v = *vi;
		vector<float> descriptor;

		feature.computeFeatureVector(descriptor,v);
		cout << "Descriptor: ";
		for(int i=0;i<descriptor.size();i++) 
			cout << descriptor[i] << " ";
		cout << endl;

		
	}

	////////////////////////////////////////////////////////////////////////////////////
	void actionScale_world_triggered() {
		cout << "downscaling inputs" << endl;
		viewer->downScaleInputs(1.0f);
	}

	////////////////////////////////////////////////////////////////////////////////////
	void actionGenerate_Random_Colors_triggered() {
		viewer->data->mesh.generateRandomColors();
	}


	////////////////////////////////////////////////////////////////////////////////////
	void actionBlur_Input_Images_triggered() {
		viewer->data->blurInputs();
	}


	////////////////////////////////////////////////////////////////////////////////////
	void actionRestrict_to_Bounding_Box_triggered() {
		BoundingBoxProps boundingProps(this,viewer->data);
		boundingProps.exec();
		viewer->updateGL();
	}
	
	////////////////////////////////////////////////////////////////////////////////////
	void on_pushInvert_pressed() {
		viewer->data->mesh.invert();
		viewer->updateGL();
	}
	
	////////////////////////////////////////////////////////////////////////////////////
	void on_pushTransformation_pressed() {
		//
		//viewer->data->mesh.shift(doubleShiftDx->value(),doubleShiftDy->value(),doubleShiftDz->value());
		MVTransformProps transf(this,viewer->data);
		transf.exec();
		viewer->updateGL();
	}
	
	////////////////////////////////////////////////////////////////////////////////////
	void on_pushUnion_pressed() {
		QString fileName = QFileDialog::getOpenFileName(this, tr("Load Mesh"),".", tr("OFF Files - Simple (*.off);; OFF Files - with Normals (*.noff);; OFF Files - with Colors (*.coff)"));
		if (fileName != "") {
			Mesh other;
			other.loadFormat((char*)fileName.toAscii().data()  ,"off");
			viewer->data->mesh.unionWith(other);
		}
		viewer->updateGL();
	}
	

	////////////////////////////////////////////////////////////////////////////////////
	void actionError_wrt_groundruth_points_triggered() {
		QString fileName = QFileDialog::getOpenFileName(this, tr("Load Groundtruth oriented Points"), ".", tr("P3D Files - Vertices with Normals(*.P3D)"));
		PointSet ps;
		if (fileName == "") return;
		bool ok;
		double completness_cutoff = QInputDialog::getDouble(this, tr("QInputDialog::getDouble()"),
													tr("Compleness Cutoff:"), viewer->data->mesh.edge_avg, 0, 10000, 5, &ok);
		if (!ok) return;
		float distError, completenessError;
		ps.load((char*)fileName.toAscii().data());		
		MeshError::computeErrors(viewer->data->mesh,ps,completness_cutoff,distError,completenessError);
		cout << "Avg. Distance Error = " << distError << endl;
		cout << "Completeness for cutoff(" << completness_cutoff << ") = " << completenessError << endl;		
		
	}

	////////////////////////////////////////////////////////////////////////////////////
	void on_pushCurvSegmentation_pressed() {
		CurvatureSegmention seg(&viewer->data->mesh,true);
		seg.segment(doubleSpinCurvSegmThreshold->value(),spinIslandThreshold->value());
		viewer->setGeometryMode('C');
		viewer->updateGL();
	}

	void on_pushEpsilonSegmentation_pressed() {
		//gseg->cleanUp(); 
		gseg->load(&viewer->data->mesh);
		viewer->setGeometryMode('C');
		viewer->updateGL();
		gseg->makeSegments(doubleSpinCurvSegmThreshold->value());
		viewer->setGeometryMode('C');
		viewer->updateGL();
	}

	void on_doubleScaleFactor_valueChanged(double newValue) {
		viewer->setScalingFactor(getScalingFactor());
		viewer->updateGL();
	}

	void on_pushGeoPatches_pressed() {
		ginteg.cleanUp(); 
		ginteg.load(&viewer->data->mesh, 2);
		fprintf(stderr, "Getting patches of max-size %f\n", doubleGeoThresh->value() );
		ginteg.patchWarshall( doubleGeoThresh->value() );
		ginteg.setColors();
		viewer->setGeometryMode('C');
		viewer->updateGL();
	}

	void on_pushGeoInteg_pressed() { 
		//if(ginteg.m == NULL) {
			ginteg.cleanUp();
			ginteg.load(&viewer->data->mesh, 1);
			fprintf(stderr, "Getting patches of max-size %f\n", doubleGeoThresh->value() );
			ginteg.patchWarshall( doubleGeoThresh->value() );
		//} 
		fprintf(stderr, "Computing Geodesic Integral \n");
		ginteg.geoIntegral(); 
		viewer->setGeometryMode('C');
		viewer->updateGL();
	}

	void on_pushExpand_pressed() {
		viewer->data->mesh.dilate(getScalingFactor());
	}

	void on_pushPerturb_pressed() {
		cout << "Noising mesh with sigma="<<getScalingFactor() << endl;
		viewer->data->mesh.noise(getScalingFactor(),'C',0);
		viewer->data->mesh.noise(getScalingFactor(),'G',0);		
		//perturb was here before
		viewer->updateGL();
	}

	void on_pushSmooth_pressed() {
		cout << "Smoothing" << endl;
		viewer->data->mesh.smooth(getScalingFactor(),6);
		viewer->updateGL();
	}

	void on_pushSmoothBilateral_pressed() {
		viewer->data->mesh.bilateralSmooth(viewer->data->mesh.edge_avg,viewer->data->mesh.edge_avg);
		viewer->updateGL();
	}

	void on_pushSmooth3_pressed() {
		viewer->data->mesh.smooth(getScalingFactor(),4);
		viewer->updateGL();
	}

	void on_pushShrink_pressed() {
		viewer->data->mesh.dilate((-1)*getScalingFactor());
		viewer->updateGL();
	}

	void on_comboCurvatureMethod_currentIndexChanged(int index) {
		viewer->data->mesh.curv_geo_size = doubleGeoThresh->value();
		viewer->data->mesh.curv_comp_method = index+1;
		viewer->data->mesh.updateMeshData();
		viewer->updateGL();
	}

/*
	void on_pushLoadPoints_pressed() {

		QString fileName = QFileDialog::getOpenFileName(this, tr("Load Point File"),
														".", tr("All Files (*.*)"));
		if (fileName != "") {
			viewer->data->mesh.createMeshFromPoints((char*)fileName.toAscii().data());
		}
		viewer->updateGL();
	}
*/	

	void on_pushEvolve_pressed() {
		cout << "Evolving mesh..";
		cout.flush();
		viewer->data->mesh.evolve(1.0);
		viewer->updateGL();		
		cout << "OK"<<endl;
	}

	void on_pushUnEvolve_pressed() {
		cout << "UnEvolving mesh..";
		cout.flush();		
		viewer->data->mesh.evolve(-1.0);
		viewer->updateGL();		
		cout << "OK"<<endl;		
	}
	

	// Viewing TAG
	
	void on_checkMeanCurv_toggled(bool state) {
		viewer->setMeanCurvature(state);
	}

	void on_checkFullScreen_toggled(bool state) {
		cout << "Toggling Full Screen" << endl;
		viewer->toggleFullScreen();
	}
	
	void on_checkSmoothShading_toggled(bool state) {
		viewer->setSmoothShading(state);
	}


	void on_checkEdges_toggled(bool state) {
		viewer->setEdges(state);
	}


	void on_checkLighting_toggled(bool state) {
		viewer->setLighting(state);
	}
	  
	void on_checkNormals_toggled(bool state) {
		viewer->setNormals(state);
	}
	
	void on_checkMotionField_toggled(bool state) {
		viewer->setMotionField(state);
	}

	void on_checkRenderTextures_toggled(bool state) {
		viewer->setTextures(state);
	}

	void on_checkDebugInfo_toggled(bool state) {
		viewer->setDebugInfo(state);
	}

	void on_spinTriangleId_valueChanged(int newValue) {
		viewer->setDebugInfoOne(checkDebugInfoOne->checkState(),spinVertexId->value(), spinTriangleId->value());
	}
	
	void on_spinVertexId_valueChanged(int newValue) {
		viewer->setDebugInfoOne(checkDebugInfoOne->checkState(),spinVertexId->value(), spinTriangleId->value());
	}

	void on_checkDebugInfoOne_toggled(bool state) {
		viewer->setDebugInfoOne(state,spinVertexId->value(), spinTriangleId->value());

	}

	void on_checkSegInter_toggled(bool state) {
		viewer->setSegmentIntersections(state);
	}

	void on_checkSegDelaunay_toggled(bool state) {
		viewer->setSegmentDelaunay(state);
	}

	void on_radioGeomNormal_pressed() {viewer->setGeometryMode('N');}
	void on_radioGeomTexture_pressed() {viewer->setGeometryMode('T');}		
	void on_radioGeomMeanCurv_pressed() {viewer->setGeometryMode('H');}
	void on_radioGeomGaussianCurv_pressed() {viewer->setGeometryMode('K');}
	void on_radioGeomWeights_pressed() {viewer->setGeometryMode('W');}		
	void on_radioGeomInter_pressed() { viewer->setGeometryMode('C');}
	
	void on_radioCullNone_pressed() {viewer->setFaceCullingMode('N'); }
	void on_radioCullFront_pressed() {viewer->setFaceCullingMode('F'); }	
	void on_radioCullBack_pressed() {viewer->setFaceCullingMode('B'); }

	void on_radioPolygonPoint_pressed() {viewer->setPolygonDrawingMode('P'); }
	void on_radioPolygonLine_pressed() {viewer->setPolygonDrawingMode('W'); }
	void on_radioPolygonFill_pressed() {viewer->setPolygonDrawingMode('F'); }

	void on_checkCameras_toggled(bool state) {
		viewer->setCameras(state);
	}
	
	////////////////////////////////////////////////////////////////////////////////////
	void on_checkDelta_toggled(bool state) {
		bool ok;
		double dt = QString("1e%1").arg(getScalingFactor()).toDouble(&ok);		
		viewer->setDeltas(state,dt);
	}	
};

#endif

#endif
