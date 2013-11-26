#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <string.h>

#include "../MVViewer/CImg_mod.h"
#include "MeshMatching.h"

int main(int argc, char** argv)
{
	
	cimg_usage("Mesh Feature Detection (MeshDOG) and Description (MeshHOG) \n Params: run the program with the -h option in order to view them.");

	char op_mode= (char)cimg_option("-op_mode",'F',"'F' - features; 'M' - matching; 'E' - evaluation; 'N' - noise; 'C' - colour with descriptor");	

	
	const char *src_mesh_file = cimg_option("-src_mesh_file","./results/dino_sparse_1.off","Source Mesh file");
	const char *dst_mesh_file = cimg_option("-dst_mesh_file","./results/dino_ring_1.off","Dest Mesh file");


	const char *src_mesh_desc_file = cimg_option("-src_mesh_desc_file","0","Source Mesh Descriptor file - features are being loaded");
	const char *dst_mesh_desc_file = cimg_option("-dst_mesh_desc_file","0","Destination Mesh Descriptor file - features are being loaded");
	
	const int curvature_rings = cimg_option("-curvature_rings", 2, "Number of rings to be considered during curvature computation (if scalar function is 1 or 2)");

	//detection
	const int detector_type = cimg_option("-detector_type", 0, "Detection: Scalar Function: 0 - color; 1 - mean curvature; 2 - Gaussian curvature; 3 - imported");
	const int detector_method = cimg_option("-detector_method", 0, "Detection Method: 0 - iterative conv.; 1 - spectral decomp; 2 - random (set detector_thresh); 3 - imported (set detector_file)");
    const char *detector_file = cimg_option("-detector_file","detections.txt","Detection: file with features, when using detection_method=3 (0-not selected; 1-selected)");
	const float detector_thresh = cimg_option("-detector_thresh", 0.2f, "Detection: top keypoints; if <1 it is a % of total vertices (0.2 corresponds to top 20%); if >1 represents the actual number of desired keypoints");
	const bool nonMaxSup= cimg_option("-non_max_sup", true, "Detection: non maxima supression");
	const double corner_thresh= cimg_option("-corner_thresh", 0.0f, "Detection: Corner Threshold: ratio btw largest and smallest eigenvalue of the hessian; 0 to disable");
	const bool scale_space=cimg_option("-scale_space", false, "Detection: use scale space");
	const int no_scales=cimg_option("-no_scales", 3, "Detection: no. of scales");

	// descriptor
	const int feature_type = cimg_option("-feature_type", 0, "Feature : Scalar Function: 0 - color; 1 - mean curvature; 2 - Gaussian curvature; 3 - Imported");
	const bool feat_useDetScale= cimg_option ( "-feat_uses_det_scale",false,"The feature is computed at the scale of the detector" );
	const float feat_no_rings  = cimg_option("-no_rings", 1.0f, "Feature : no. of rings");
	const bool treat_no_rings_as_surface_percentage = cimg_option("-rings_as_percentage", true, "Feature : Treat the number of rings as surface percentage");		
	const int feat_no_bins_centroid  = cimg_option("-no_bins_centroid", 36, "Feature : no. of bins used to compute centroid");
	const int feat_no_bins_groups  = cimg_option("-no_bins_groups", 4, "Feature : no. of bins used to split in groups");
	const int feat_no_bins_orientations  = cimg_option("-no_bins_orientations", 8, "Feature : no. of orientations within each group");
	const double feat_spatial_influence  = cimg_option("-spatial_influence", 0.3,"Feature: spatial influence");
	const bool feat_compute_all= cimg_option ( "-feat_compute_all",false,"Feature: Compute features at all vertices" );
	
	//matching
	const double matching_2nd_ratio  = cimg_option("-matching_2nd_ratio", 0.7,"Matching: the ratio threshold between the 1st and 2nd best match");
	const char *groundtruth = cimg_option("-groundtruth","0","Evaluation: 0 - no groundtruth; 1 - same no.vert; 2 - meshes aligned; 3- filename with matches");
	const char *matches_dst_src_file = cimg_option("-matches_dst_src_file","0","Destination / Source Matches - used as groundtruth when in op_mode='E' and groundtruth=3");

	// output	
	const char *save_output_file = (char*)cimg_option("-save_output_file","./results.txt","Output file");
	const char *save_output_format = (char*)cimg_option("-save_output_format","sum","Output file type (sum - summary;det - detailed; idx - indices)");


	// noise 
	bool noise_data								= cimg_option("-noise",false,"Noise Data");	
	const float noise_sigma_colour				= cimg_option("-noise_sigma_colour",0.0f,"Noise Sigma: a Value between 0 and 1");
	const float noise_sigma_colour_shot			= cimg_option("-noise_sigma_colour_shotnoise",0.0f,"Noise Sigma: a Value between 0 and 1");	
	
	const float noise_sigma_geometry			= cimg_option("-noise_sigma_geom_noise",0.0f,"Noise Sigma: a Value between 0 and 1");
	const float noise_sigma_geometry_shot		= cimg_option("-noise_sigma_geom_shotnoise",0.0f,"Noise Sigma: a Value between 0 and 1");	
	const float noise_sigma_geom_rotate			= cimg_option("-noise_sigma_geom_rotate",0.0f,"Noise Sigma: a Value between 0 and 1");
	const float noise_sigma_geom_scale			= cimg_option("-noise_sigma_geom_scale",0.0f,"Noise Sigma: a Value between 0 and 1");	
	const float noise_sigma_geom_local_scale	= cimg_option("-noise_sigma_geom_local_scale",0.0f,"Noise Sigma: a Value between 0 and 1 (10 max local operations)");	
	const float noise_sigma_geom_sampling		= cimg_option("-noise_sigma_geom_sampling",0.0f,"Noise Sigma: a Value between 0 and 1 (10 max sub-divisions)");
	const float noise_sigma_geom_holes			= cimg_option("-noise_sigma_geom_holes", 0.0f, "Noise Sigma: a Value between 0 and 1");
	const float noise_sigma_geom_micro_holes	= cimg_option("-noise_sigma_geom_micro_holes", 0.0f, "Noise Sigma: a Value between 0 and 1");
	const float noise_sigma_geom_topology		= cimg_option("-noise_sigma_geom_topology", 0.0f, "Noise Sigma: a Value between 0 and 1");
	

	if (cimg_option("-h",(const char *)0,0)) return 0;
	bool save_outputs = true;

	// Create the algorithm
	MeshMatching alg;
	alg.setParams(src_mesh_file, dst_mesh_file, curvature_rings, feat_no_rings, treat_no_rings_as_surface_percentage, feat_no_bins_centroid, feat_no_bins_groups , feat_no_bins_orientations , feat_spatial_influence, feat_compute_all, matching_2nd_ratio, matches_dst_src_file, save_outputs, save_output_file, save_output_format, groundtruth, nonMaxSup, scale_space, no_scales,  feat_useDetScale, corner_thresh, feature_type, detector_type, detector_method, detector_file, detector_thresh, src_mesh_desc_file, dst_mesh_desc_file, op_mode,  noise_data, noise_sigma_colour, noise_sigma_colour_shot, noise_sigma_geometry, noise_sigma_geometry_shot, noise_sigma_geom_rotate, noise_sigma_geom_scale, noise_sigma_geom_local_scale, noise_sigma_geom_sampling, noise_sigma_geom_holes, noise_sigma_geom_micro_holes, noise_sigma_geom_topology);	
	alg.run();
}
