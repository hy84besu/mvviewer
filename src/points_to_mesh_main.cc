#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include "CImg_mod.h"
#include <string.h>

#include "PointsToMeshWindow.h"


#include "MVData.h"
#include <QApplication>
#include <QCoreApplication>
int main(int argc, char** argv)
{
	
	cimg_usage("Multi-Stereo reconstruction\n Params: run the program with the -h option in order to view them.");

	const char *dst_points_file = cimg_option("-dst_points_file","./datasets/verify/franco-knots.off","Destination Points (with normal)");
	const char *src_mesh_file = cimg_option("-src_mesh_file","./datasets/verify/init.off","Source Mesh file");
	const char *src_vect_file = cimg_option("-src_vec_file","","Source Mesh Vector Field file");	
	const bool reposition = cimg_option("-reposition",true,"Re-position the source such that overlaps with dest.");	
	const bool use_gui = cimg_option ( "-gui",true,"true/false" );

	


	const int init_remeshes  = cimg_option("-init_remeshes", 5,"NoGUI Alg - Initial subdivisions");
	const double init_bbox_scale  = cimg_option("-init_rescale", 1.0,"NoGUI Alg - Initial relative scaling");		
	const double dt = cimg_option("-dt",0.5,"NoGUI Alg - Dt");
	const int iter  = cimg_option("-iter",999,"NoGUI Alg - iterations");
	const int scales  = cimg_option("-scales",3,"NoGUI Alg - scales");
	const double smoothing  = cimg_option("-smoothing",0.10,"NoGUI Alg - smoothing");
	bool autoStop = cimg_option("-auto_stop",true,"NoGUI Alg - auto stop from iterations");	
	bool save_outputs = cimg_option("-save_outputs",false,"NoGUI Alg - save outputs");	
    char *save_output_prefix = (char*)cimg_option("-save_output_prefix","./intermediate_outputs","NoGUI Alg - Path Prefix when saving outputs");


    if (cimg_option("-h",(const char *)0,0)) return 0;
	
	MVData data;
	data.mesh.loadFormat(src_mesh_file,true);

	if ( use_gui==false) { // no gui
		save_outputs = true;
	}
	
	// Create the algorithm
	PointsToMesh alg(&data);
	alg.setParams(dst_points_file,src_mesh_file, src_vect_file, reposition, init_remeshes, init_bbox_scale, dt, iter, scales, smoothing, autoStop, save_outputs, save_output_prefix);

	// Instantiate the viewer.
	
	if ( use_gui==false) { // no gui
		// Read command lines arguments.
		QApplication application ( argc,argv );
		printf ( "Running without GUI.\n" );
		alg.execute();
	}
	else {
		// Read command lines arguments.
		QApplication application(argc,argv);
		PointsToMeshWindow v;
		v.setAlg(&alg);
		
		// Make the viewer window visible on screen.
		v.show();
		// Set the viewer as the application main widget.
#if QT_VERSION < 0x040000
		application.setMainWidget(&v);
#endif
		application.connect( &application, SIGNAL(lastWindowClosed()), &application, SLOT(quit()) );
		// Run main loop.
		return application.exec();
	}
}
