#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include "CImg_mod.h"
#include <string.h>

#include "MeshMatchingWindow.h"


#include "MVData.h"
#include <QApplication>
#include <QCoreApplication>
int main(int argc, char** argv)
{
	
	cimg_usage("Multi-Stereo reconstruction\n Params: run the program with the -h option in order to view them.");

	const char *src_mesh_file = cimg_option("-src_mesh_file","./results/dino_sparse_1.off","Source Mesh file");
	const char *dst_mesh_file = cimg_option("-dst_mesh_file","./results/dino_ring_1.off","Dest Mesh file");


	const char *src_mesh_desc_file = cimg_option("-src_mesh_desc_file","0","Source Mesh Descriptor file - features are being loaded");
	const char *dst_mesh_desc_file = cimg_option("-dst_mesh_desc_file","0","Destination Mesh Descriptor file - features are being loaded");

	const bool use_gui = cimg_option ( "-gui",true,"true/false" );

	const int detector_type = cimg_option("-detector_type", 0, "Detection: 0 - color; 1 - mean curvature; 2 - gaussian curvature; 3 - color + mean curvature");
	const float detector_thresh = cimg_option("-detector_thresh", 0.2f, "Detection: top matches (0.2 corresponds to top 20%)");
	const bool nonMaxSup= cimg_option("-non_max_sup", true, "Detection: non maxima supression");
	const double corner_thresh= cimg_option("-corner_thresh", 0.0f, "Detection: Corner Threshold: ratio btw largest and smallest eigenvalue of the hessian; 0 to disable");
	const bool scale_space=cimg_option("-scale_space", false, "Detection: use scale space");
	const int no_scales=cimg_option("-no_scales", 3, "Detection: no. of scales");

	const int feature_type = cimg_option("-feature_type", 0, "Feature : 0 - color; 1 - mean curvature; 2 - gaussian curvature");
	const int feat_no_rings  = cimg_option("-no_rings", 1, "Feature : no. of rings");
	const bool treat_no_rings_as_surface_percentage = cimg_option("-rings_as_percentage", true, "Feature : Treat the number of rings as surface percentage");		
	const int feat_no_bins_centroid  = cimg_option("-no_bins_centroid", 36, "Feature : no. of bins used to compute centroid");
	const int feat_no_bins_groups  = cimg_option("-no_bins_groups", 4, "Feature : no. of bins used to split in groups");
	const int feat_no_bins_orientations  = cimg_option("-no_bins_orientations", 8, "Feature : no. of orientations within each group");
	const double feat_spatial_influence  = cimg_option("-spatial_influence", 0.3,"Feature: spatial influence");

	const double matching_2nd_ratio  = cimg_option("-matching_2nd_ratio", 0.7,"Matching: the ratio threshold between the 1st and 2nd best match");

	const char *groundtruth = cimg_option("-groundtruth","0","Evaluation: 0 - no groundtruth; 1 - same no.vert; 2 - meshes aligned; filename with matches");

			
	bool save_outputs = cimg_option("-save_outputs",true,"save outputs");	
	const char *save_output_file = (char*)cimg_option("-save_output_file","./matches.txt","Output file");
	const char *save_output_format = (char*)cimg_option("-save_output_format","sum","Output file type (sum - summary;det - detailed; idx - indices)");


	bool noise_data = cimg_option("-noise",false,"Noise Data");	
	const float noise_sigma_geometry = cimg_option("-noise_sigma_geometry",0.02f,"Noise Sigma: a Value between 0 and 1");			
	const float noise_sigma_colour = cimg_option("-noise_sigma_colour",0.02f,"Noise Sigma: a Value between 0 and 1");

	bool features_only= cimg_option("-features_only",false,"Computes and outputs only the features of the src_mesh_file");	


	//if (cimg_option("-h",(const char *)0,0)) return 0;
	
	MVData data;
	if ( use_gui==false) // no gui
		save_outputs = true;
	else {
		data.mesh.loadFormat(src_mesh_file,true);
		Mesh m2;
		m2.loadFormat(dst_mesh_file,true);
		data.mesh.unionWith(m2);
	}
	
	// Create the algorithm
	MeshMatching alg(&data);
	
	alg.setParams(src_mesh_file, dst_mesh_file, feat_no_rings, treat_no_rings_as_surface_percentage, feat_no_bins_centroid, feat_no_bins_groups , feat_no_bins_orientations , feat_spatial_influence, matching_2nd_ratio, save_outputs, save_output_file, save_output_format, groundtruth, nonMaxSup, scale_space, corner_thresh, feature_type, detector_type, detector_thresh, src_mesh_desc_file, dst_mesh_desc_file, features_only, noise_data, noise_sigma_geometry, noise_sigma_colour);

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
		MeshMatchingWindow v;
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
