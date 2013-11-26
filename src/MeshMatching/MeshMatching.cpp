/*
 *  MeshMatching.cpp
 *  src
 *
 *  Created by Andrei Zaharescu on 12/07/07.
 *  Copyright 2007 __MyCompanyName__. All rights reserved.
 *
 */

#include "MeshMatching.h"
#include "Histogram.h"
#include "FeatureGradientHistogram.h"
#include "ColorMap.h"
#include <CGAL/Timer.h>


std::ostream& operator<<(std::ostream& os, const VertexDescriptor& vd) {
	os << "Vertex [" << vd.vertex->id << "] -> ";
	for (int d = 0; d < vd.descriptor.size(); ++d)
		os << ' ' << vd.descriptor[d];
	return os << ' ';
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
// Destructeur
MeshMatching::MeshMatching(MVData* _data) : data(_data) {
	is_init = false;
	alg_saveOutput = false;
	alg_scaleSpace = true;
	alg_curvNoRings = 5;
	alg_detType=0;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
// Destructeur
MeshMatching::~MeshMatching() {
	if (is_init)
		EndAlg();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// end of story
void MeshMatching::EndAlg() {
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
void MeshMatching::setParams(const char* src_mesh_file, const char* dst_mesh_file, int feat_no_rings, bool treat_no_rings_as_surface_percentage, int feat_no_bins_centroid, int feat_no_bins_groups , int feat_no_bins_orientations , double feat_spatial_influence, double matching_2nd_ratio, bool save_outputs, const char * save_output_filename, const char * save_output_format, const char * groundtruth, bool non_max_sup, bool scale_space, float corner_thresh, int feature_type, int detector_type, float det_thresh, const char * src_mesh_desc_file, const char * dst_mesh_desc_file, bool features_only, bool noise_data, float noise_sigma_geometry, float noise_sigma_colour) {

	is_init = true;

	alg_featType = feature_type;
	alg_featNoRings = feat_no_rings;
	alg_treatNoRingsAsSurfacePercentage=treat_no_rings_as_surface_percentage;
	alg_featNoBinsCentroid = feat_no_bins_centroid;
	alg_featNoBinsGroups = feat_no_bins_groups;
	alg_featNoBinsOrientations = feat_no_bins_orientations;
	alg_featSpatialInfluence = feat_spatial_influence;

	alg_detType = detector_type;
	alg_detThresh = det_thresh;
	alg_matchingBestMatchesRatio = matching_2nd_ratio;
	alg_nonMaxSup = non_max_sup;
	alg_scaleSpace = scale_space;
	alg_cornerThresh = corner_thresh;

	alg_saveOutput = save_outputs;
	alg_featuresOnly = features_only;

	alg_noise=noise_data;
	alg_noiseSigmaGeometry=noise_sigma_geometry;
	alg_noiseSigmaColour=noise_sigma_colour;

	
	strcpy(alg_srcMeshFile,src_mesh_file);
	strcpy(alg_dstMeshFile,dst_mesh_file);
	strcpy(alg_srcMeshDescFile,src_mesh_desc_file);
	strcpy(alg_dstMeshDescFile,dst_mesh_desc_file);
	strcpy(alg_saveOutputFile,save_output_filename);
	strcpy(alg_saveOutputFormat,save_output_format);
	strcpy(alg_groundtruthFile,groundtruth);


}


//////////////////////////////////////////////////////////////////////////////////////////////////////
void MeshMatching::importFeaturesFromFile(const char* filename, vector<Vertex>& vertex_storage, vector<VertexDescriptor>& output_matches) {

	ifstream ifs(filename);
	int no_matches, descriptor_size;
	ifs >> no_matches;
	ifs >> descriptor_size;

	cout << "* Importing " << no_matches << " matches from "<<filename << endl;

	for(int i=0;i<no_matches;i++) {
		Vertex* v = new Vertex();
		vector<float> desc;
		float tmpF;
		int id;
		ifs >> id;
		v->id=id;
		for(int j=0;j<descriptor_size;j++) {
			ifs>>tmpF;
			desc.push_back(tmpF);
		}
		desc[0]=0; desc[1]=0;

		float a,b,c;
		ifs >> a >> b >> c;
		v->point()=Point(a,b,c);
		ifs >> a >> b >> c;
		v->normal()=Vector(a,b,c);
		ifs >> a >> b >> c;
		float rgb[3];
		ColorMap::RGBtoHSV(a,b,c,rgb);
		v->set_color(rgb[0],rgb[1],rgb[2]);

		output_matches.push_back(VertexDescriptor());
		output_matches.back().setVertex(*v);
		output_matches.back().setDescriptor(desc);
		
/*		cout << "id=" << id << ": " << output_matches.back() << endl;
		cout << "vertex: " << output_matches.back().vertex->point() << endl;
		cout << "normal: " << output_matches.back().vertex->normal() << endl;
		cout << "-----------------------------------------" << endl;
*/	} 

}


//////////////////////////////////////////////////////////////////////////////////////////////////////
void MeshMatching::saveDescriptorList(vector<VertexDescriptor> &vec_vd, ostream &out, bool pointsOnly) {

	if (pointsOnly==false)
		out << vec_vd.size()  << " " << vec_vd[0].descriptor.size() << endl;
	for(int i=0;i<vec_vd.size();i++) {
		if (pointsOnly) {
			out << vec_vd[i].vertex->point() << endl;
		}
		else {
			out << i << " ";
			for(int j=0;j<vec_vd[i].descriptor.size();j++)
				out << vec_vd[i].descriptor[j] << " ";
	
			float hsv[3];
			float* color = vec_vd[i].vertex->color;
			ColorMap::RGBtoHSV(color[0],color[1],color[2],hsv);
			out << hsv[0] << " " << hsv[1] << " " << hsv[2] << " ";
			out << vec_vd[i].vertex->point() << " ";
			out << vec_vd[i].vertex->normal();
			out << endl;
		}
	}
}

/*
//////////////////////////////////////////////////////////////////////////////////////////////////////
float MeshMatching::getGradient(Vertex &v, int mode) {
	if (mode==0) return (float) v_norm(Mesh::computeVertexGradient(v,Mesh::Qual_Color_Deriv));
	if (mode==1) return (float) v_norm(Mesh::computeVertexGradient(v,Mesh::Qual_Mean_Curv_Deriv));
	if (mode==2) return (float) v_norm(Mesh::computeVertexGradient(v,Mesh::Qual_Gaussian_Curv_Deriv));
}
*/

//////////////////////////////////////////////////////////////////////////////////////////////////////
void MeshMatching::detectComputeFeatures(Mesh &mesh, vector<VertexDescriptor> &vec_vd) {
	int bins=1000;
	int v_id=0;
	float percentile=alg_detThresh;

	CGAL::Timer time_elapsed;
	time_elapsed.start();

	float quality_min=std::numeric_limits<float>::max();
	float quality_max=std::numeric_limits<float>::min();
	Histogramf h;
	float lower_bound, upper_bound;

	
//	cout << " - computing " << alg_detThresh/2 << "th and " << 1-alg_detThresh/2 << "th percentile." <<endl;
	cout << " - computing " << alg_detThresh << "th percentile." <<endl;	
	
	if (alg_detType==0) mesh.setVertexQuality(Mesh::Qual_Color_Deriv);
	if (alg_detType==1) mesh.setVertexQuality(Mesh::Qual_Mean_Curv_Deriv);	
	if (alg_detType==2) mesh.setVertexQuality(Mesh::Qual_Gaussian_Curv_Deriv);		
	//computes the gradients of the quality measures and sets the quality as the gradient magnitude

/*	
	for (Vertex_iterator vi = mesh.p.vertices_begin(); vi!=mesh.p.vertices_end(); vi++) {
		vi->quality()=getGradient(*vi,alg_detType);
	}
*/
	
	// get max and min
	for (Vertex_iterator vi = mesh.p.vertices_begin(); vi!=mesh.p.vertices_end(); vi++) {
		if (quality_min > vi->quality())
			quality_min = vi->quality();
		if (quality_max < vi->quality())
			quality_max = vi->quality();
	}
	h.SetRange(quality_min,quality_max,bins);


	vector<float> qual_elems;
	vector<bool> is_chosen_elems;
	
	if (alg_scaleSpace) {
		
		if (alg_detType==0) mesh.setVertexQuality(Mesh::Qual_Color);
		if (alg_detType==1) mesh.setVertexQuality(Mesh::Qual_Mean_Curv);	
		if (alg_detType==2) mesh.setVertexQuality(Mesh::Qual_Gaussian_Curv);		

		int no_scales=3;

		int no_convolutions=1;
		int cur_convs_per_scale=3;

		for(int i=1;i<no_scales;i++) {
			no_convolutions+=cur_convs_per_scale;
			cur_convs_per_scale*=2;
		}

		//perform the convolutions
		for(int i=0;i<no_convolutions;i++) {
			cout << " - convolving " << i+1 << "/" << no_convolutions << " " ;
			mesh.convolve(1.25,true);
		}

		//reset the flags
		for (Vertex_iterator vi = mesh.p.vertices_begin(); vi!=mesh.p.vertices_end(); vi++) {
//			is_chosen_elems.push_back(false);
			vi->flag[1]=false;
		}



		cout << "Sample: ";
		Vertex &vvv=*mesh.p.vertices_begin();
		for(int i=0;i<no_convolutions;i++) 
			cout << vvv.values[i] << " ";
		cout << endl;
		
		cout << "size of values << " << vvv.values.size() << endl;
		
		
		for(int i=0;i<no_convolutions;i++) 
			for (Vertex_iterator vi = mesh.p.vertices_begin(); vi!=mesh.p.vertices_end(); vi++) {
				if (vi->flag[1]==false) {

					vector<pair<Vertex*,int> > neighs = Mesh::getNeighbourhood(*vi,1);
					bool is_maxima=true;
					float the_sign=(vi->values[i] - neighs[0].first->values[i]);
					for (int j=0;j<neighs.size();j++) {
						if (the_sign*(vi->values[i] - neighs[j].first->values[i])<=0) {
							is_maxima = false;
							break;
						}
						if ((i>0) && (the_sign*(vi->values[i] - neighs[j].first->values[i-1])<=0)) {
							is_maxima = false;
							break;
						}
						if ((i<(no_convolutions-1)) && (the_sign*(vi->values[i] - neighs[j].first->values[i+1])<=0)) {
							is_maxima = false;
							break;
						}
					}
					
					if (is_maxima && (i>0) && (the_sign*(vi->values[i]-vi->values[i-1])<=0)) 
						is_maxima=false;
					if (is_maxima && (i<(no_convolutions-1))  && (the_sign*(vi->values[i]-vi->values[i+1])<=0)) 
						is_maxima=false;

					
					if (is_maxima) {
						vi->flag[1]=true;
						vi->quality()=abs(vi->values[i]);
					}
				}
			}

		for (Vertex_iterator vi = mesh.p.vertices_begin(); vi!=mesh.p.vertices_end(); vi++) {
			is_chosen_elems.push_back(vi->flag[1]);
			if (vi->flag[1]) {
				h.Add(vi->quality());
				qual_elems.push_back(vi->quality());
			}
		}

	}
	else {
		//add values
		for (Vertex_iterator vi = mesh.p.vertices_begin(); vi!=mesh.p.vertices_end(); vi++) {
			//non maxima suppression
			vector<pair<Vertex*,int> > neighs = Mesh::getNeighbourhood(*vi,1);
			bool is_chosen=true;
			if (alg_nonMaxSup) {
				for (int i=0;i<neighs.size();i++)
					if (vi->quality() <= neighs[i].first->quality()) {
						is_chosen = false;
						break;
					}
			}

/*			if (is_chosen && (alg_cornerThresh>0) )
				is_chosen=mesh.isCorner(*vi,alg_cornerThresh);
*/
			is_chosen_elems.push_back(is_chosen);
			if (is_chosen) {
				h.Add(vi->quality());
				qual_elems.push_back(vi->quality());
				vi->flag[1]=true;
			}
			else
				vi->flag[1]=false;			
			//			vi->quality()=std::numeric_limits<float>::min();
		}
		
	} // no scale space
		

//	float local_detThresh = alg_detThresh*mesh.p.size_of_vertices()/qual_elems.size();
//	if (local_detThresh > 1 ) local_detThresh=1;
//	lower_bound = h.Percentile(local_detThresh);
//	upper_bound = h.Percentile(1.0f-alg_detThresh);
	

	int nth_elem = (int)qual_elems.size()-(int)round(alg_detThresh*mesh.p.size_of_vertices());
	int max_nth=20;
	if (nth_elem > max_nth)
		nth_elem = max_nth;
	else {
		if (nth_elem<0) nth_elem=0;
	}
	std::nth_element(qual_elems.begin(),qual_elems.begin()+nth_elem,qual_elems.end()); //-1
	
	//	cout << qual_elems[nth_elem] << " "  << endl;
	//	std::sort(qual_elems.begin(), qual_elems.end());
	//	cout << "------------" << endl;
	//	std::copy(qual_elems.begin(), qual_elems.end(), ostream_iterator<float>(cout, " "));
	//	cout << "------------" << endl;
	//	cout << "elem[" << nth_elem << "]=" << qual_elems[nth_elem] << " "  << endl;
	if (qual_elems.size() > 0)
		upper_bound = qual_elems[nth_elem];
	//	cout << "upper bound=" << upper_bound << endl;
	
	/*	
		nth_elem = std::max(20,(int)qual_elems.size()-(int)round((1.0f-alg_detThresh/2)*mesh.p.size_of_vertices()));
		std::nth_element(qual_elems.begin(),qual_elems.begin()+nth_elem-1,qual_elems.end());
		lower_bound = qual_elems[nth_elem];
		*/	
	
	//set the final list of chosen vertices
	v_id=0;		

	int local_count=0;
	int counter_extrema=0;
	int counter_threshold=0;
	int counter_corner=0;
	for (Vertex_iterator vi = mesh.p.vertices_begin(); vi!=mesh.p.vertices_end(); vi++, v_id++) {
		//		if ((upper_bound > vi->quality()) && (lower_bound < vi->quality())) {
		if (is_chosen_elems[v_id])
			counter_extrema++;

		if ((upper_bound > vi->quality()) && (vi->flag[1])) {			
			is_chosen_elems[v_id] = false;
		}	

		if (is_chosen_elems[v_id])
			counter_threshold++;

		if (is_chosen_elems[v_id] && (alg_cornerThresh>0) )
			is_chosen_elems[v_id]=mesh.isCorner(*vi,alg_cornerThresh);

		if (is_chosen_elems[v_id])
			counter_corner++;
	}
	cout << " - detection (1): extrema of gradient; points kept=" <<  counter_extrema << endl;
	cout << " - detection (2): magnitude thresholding; points kept=" << counter_threshold << endl;
	if (alg_cornerThresh>0)
		cout << " - detection (3): corner thresholding; points kept = " << counter_corner  << endl;
	else
		cout << " - detection (3): no corner thresholding; points kept = " << counter_corner  << endl;


	if (alg_scaleSpace) {	
		if (alg_detType==0) mesh.setVertexQuality(Mesh::Qual_Color_Deriv);
		if (alg_detType==1) mesh.setVertexQuality(Mesh::Qual_Mean_Curv_Deriv);	
		if (alg_detType==2) mesh.setVertexQuality(Mesh::Qual_Gaussian_Curv_Deriv);	
	}

	int local_ring_size= alg_featNoRings;
	if (alg_treatNoRingsAsSurfacePercentage) {
		local_ring_size = round(sqrt(local_ring_size*mesh.area_avg*mesh.p.size_of_facets()/100.0)/mesh.edge_avg);
		cout << " - estimating ring size for " << alg_featNoRings << "% = " << local_ring_size<< endl;	
		
	}
	if (alg_featType==0) mesh.setVertexQuality(Mesh::Qual_Color_Deriv);
	if (alg_featType==1) mesh.setVertexQuality(Mesh::Qual_Mean_Curv_Deriv);	
	if (alg_featType==2) mesh.setVertexQuality(Mesh::Qual_Gaussian_Curv_Deriv);		
	
	
	// populate the vertex description vectors
	FeatureGradientHistogram feature(0, local_ring_size, alg_featNoBinsCentroid, alg_featNoBinsGroups, alg_featNoBinsOrientations, alg_featSpatialInfluence);;

	if (alg_featType<=2) {
		feature=FeatureGradientHistogram(alg_featType, local_ring_size, alg_featNoBinsCentroid, alg_featNoBinsGroups, alg_featNoBinsOrientations, alg_featSpatialInfluence);
	}else if (alg_featType==3) {
		mesh.setVertexQuality(Mesh::Qual_Color_Deriv);
		feature=FeatureGradientHistogram(0, local_ring_size, alg_featNoBinsCentroid, alg_featNoBinsGroups, alg_featNoBinsOrientations, alg_featSpatialInfluence);
	}

	
	vector<float> descriptor;
	VertexDescriptor vd;

	cout << " - computing descriptors for selected interest points for mesh" << endl;

	

	v_id=0;
	for (Vertex_iterator vi = mesh.p.vertices_begin(); vi!=mesh.p.vertices_end(); vi++, v_id++) {
		if (is_chosen_elems[v_id]) {
			feature.computeFeatureVector(descriptor,*vi);
			vd.setVertex(*vi);
			vd.setDescriptor(descriptor);
			vec_vd.push_back(vd);
		}
	}
	
	if (alg_featType>2) { // gotto make another run to compute descriptors for a new feature
		if (alg_featType==3) {
			mesh.setVertexQuality(Mesh::Qual_Mean_Curv_Deriv);
			feature=FeatureGradientHistogram(1, local_ring_size, alg_featNoBinsCentroid, alg_featNoBinsGroups, alg_featNoBinsOrientations, alg_featSpatialInfluence);
		}
		for(int i=0;i<vec_vd.size();i++) {
			feature.computeFeatureVector(descriptor,*vec_vd[i].vertex);
			vec_vd[i].appendDescriptor(descriptor);
		}
	}
		
	cout << " - elapsed  "  << ( ( int ) time_elapsed.time() ) /60 << ":" << ( ( int ) time_elapsed.time() ) %60 << endl;	
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
// the algorithm
void MeshMatching::run() {

	int curvature_rings  = 2;
	vector<VertexDescriptor> vec_vd1;
	vector<VertexDescriptor> vec_vd2;

//	float bbox_max_movement_percentage=0.3;
	bool evaluate_matches=false;
	
	cout << " - non maximum supression = " << alg_nonMaxSup << endl;
	cout << " - detector type = " << alg_detType<< endl;
	cout << " - feature type = " << alg_featType<< endl;
	if ((alg_featType!=0) || (alg_detType!=0)) {
		mesh1.curv_comp_method = Mesh::Curv_Dong;
		mesh1.curv_comp_neigh_rings = curvature_rings;
	}
	mesh1.loadFormat(alg_srcMeshFile,false);
	if (alg_noise) {
			mesh1.noise(alg_noiseSigmaGeometry,'G');
			mesh1.noise(alg_noiseSigmaColour,'C');		
			cout << " - noising data with sigma_colour=" << alg_noiseSigmaColour << " and sigma_geom=" << alg_noiseSigmaGeometry << endl;
	}

/*	float *bbox = mesh1.getBoundingBox();
	float bbox_max_dim = max(abs(bbox[3]-bbox[0]),max(abs(bbox[4]-bbox[1]),abs(bbox[5]-bbox[2])));
	cout << " - estimating ring size with bounding box (10%)=" << round(bbox_max_dim*0.1/mesh1.edge_avg) << endl;
*/
		

	if (strcmp(alg_srcMeshDescFile,"0")==0)
		detectComputeFeatures(mesh1,vec_vd1);
	else 
		importFeaturesFromFile(alg_srcMeshDescFile, loaded_vertices_mesh1, vec_vd1);



/*
	int vertex_no=142;
	Vertex_iterator vi = mesh1.p.vertices_begin();
	for(int i=0;i<vertex_no;i++) vi++;
	descriptor=feature.computeFeatureVector(*vi);
	vd.setVertex(*vi);
	vd.setDescriptor(descriptor);
	cout << vd << endl<< endl;

	Vertex_iterator vi2 = mesh2.p.vertices_begin();
	for(int i=0;i<vertex_no;i++) vi2++;
	descriptor=feature.computeFeatureVector(*vi2);
	vd.setVertex(*vi2);
	vd.setDescriptor(descriptor);
	cout << vd << endl << endl;
	return;
*/

	if (alg_featuresOnly==true) { //compute features on the 1st mesh and exit
		cout << " - descriptors: no(mesh1)=" << vec_vd1.size() << endl;
		//saveOutput
		if (alg_saveOutput) {
		
				cout << " - saving to " << alg_saveOutputFile << " (first mesh keypoints only)" <<endl;
				ofstream fmatches(alg_saveOutputFile);
				if (!fmatches) {
					std::cerr << "Error : Unable to write '" << alg_saveOutputFile << "'" << std::endl;
					return;
				}
				saveDescriptorList(vec_vd1,fmatches);

				char alg_FeaturePointsFile[strlen(alg_saveOutputFile)+30];
				sprintf(alg_FeaturePointsFile,"%s.points",alg_saveOutputFile);

				ofstream fmatches2(alg_FeaturePointsFile);
				if (!fmatches2) {
					std::cerr << "Error : Unable to write '" << alg_FeaturePointsFile << "'" << std::endl;
					return;
				}
				saveDescriptorList(vec_vd1,fmatches2,true);
				

		}
		return;
	}


	if ((alg_featType!=0) || (alg_detType!=0)) {
		mesh2.curv_comp_method = Mesh::Curv_Dong;
		mesh2.curv_comp_neigh_rings = curvature_rings;
	}

	mesh2.loadFormat(alg_dstMeshFile,false);

	if (strcmp(alg_dstMeshDescFile,"0")==0)
		detectComputeFeatures(mesh2,vec_vd2);
	else
		importFeaturesFromFile(alg_dstMeshDescFile, loaded_vertices_mesh2, vec_vd2);


	cout << "* Matching" << endl;
	if (mesh1.p.size_of_vertices() == mesh2.p.size_of_vertices()) {
		cout << " - setting groundtruth to 1" << endl;
		strcpy(alg_groundtruthFile,"1");
	}

	if (strcmp(alg_groundtruthFile,"0")==0) {
		cout << " - no groundtruth computation." << endl;
	}
	else if (strcmp(alg_groundtruthFile,"1")==0) {
		evaluate_matches=true;
		cout << " - inferring groundtruth info by assuming the same vertex order in both meshes." << endl;

		Vertex_iterator vi2 = mesh2.p.vertices_begin();
		for (Vertex_iterator vi = mesh1.p.vertices_begin(); vi!=mesh1.p.vertices_end(); vi++,vi2++) {
			matches_groundtruth[vi->point()] = vi2->point();
		}
	}
	else if (strcmp(alg_groundtruthFile,"2")==0) {
		evaluate_matches=true;
		cout << " - assuming that the two meshes are similar and aligned." << endl;
		for (Vertex_iterator vi = mesh1.p.vertices_begin(); vi!=mesh1.p.vertices_end(); vi++) {
			matches_groundtruth[vi->point()] = vi->point();
		}
	}
	else {
		cout << " - groundtruth file option not implemented yet." << endl;
	}


	cout << " - descriptors: no(mesh1)=" << vec_vd1.size() << " and no(mesh2)=" << vec_vd2.size() << endl;
	matches.clear();
	matches_index.clear();
	cout << " - performing matching "; cout.flush();

	
//	float max_movement=bbox_max_movement_percentage*bbox_max_dim; 
	
	for (int i=0;i<vec_vd1.size();i++) {
		float min_dist, min_2nd_dist;
		int min_pos;
		
		//find closest match
		min_dist=MAXFLOAT;
		for (int j=0;j<vec_vd2.size();j++) {
			float dist=vec_vd1[i].distanceTo(vec_vd2[j],1);
			if (min_dist>dist) { min_dist=dist; min_pos=j;}
		}

		//find closest match back
		float min_dist_back=MAXFLOAT;
		int min_pos_back=1;
		for (int ii=0;ii<vec_vd1.size();ii++) {
			float dist=vec_vd1[ii].distanceTo(vec_vd2[min_pos],1);
			if (min_dist_back>dist) { min_dist_back=dist; min_pos_back=ii;}
		}

			
		//find 2nd closest match
		min_2nd_dist=MAXFLOAT;
		for (int j=0;j<vec_vd2.size();j++) {
			float dist=vec_vd1[i].distanceTo(vec_vd2[j],1);
			if ((min_2nd_dist>dist) && (j!=min_pos)) { min_2nd_dist=dist;}
		}

		
		bool element_found=true;		
		//2nd best matching ratio criteria
		if (min_dist > alg_matchingBestMatchesRatio*min_2nd_dist) element_found=false;
	
		if (min_pos_back!=i) element_found=false;
		
		
		//		if (element_found && (max_movement<v_norm(vec_vd1[i].vertex->point()-vec_vd2[min_pos].vertex->point())))  element_found=false;



		if (element_found)  {
			matches.push_back(vdpair(vec_vd1[i],vec_vd2[min_pos]));
			matches_index.push_back(pair<int,int>(i,min_pos));
		}
	}
	cout << "(" << matches.size() << " matches found)" << endl;

	
	//match evaluation - uses groundtruth!!
	Histogramf error_h;	
	int bins=20;	
	if (evaluate_matches) {
		cout << " - match error hist: " ;
		
		//error_h.SetRange(-mesh1.edge_avg,mesh1.edge_avg*(bins-1),bins);
		error_h.SetRange(0,mesh1.edge_avg*(bins),bins);
		
		for(int i=0;i<matches.size();i++) {
			Point closest_point = mesh1.closestPoint(matches[i].first.vertex->point());
			float match_error = v_norm(matches_groundtruth[closest_point] - matches[i].second.vertex->point());

			float epsilon=0.000000000001;
			error_h.Add(std::max(std::min(match_error,(float)mesh1.edge_avg*(bins)),epsilon));
			//cout << i << " " << match_error << endl;
		}
		for(int i=0;i<bins;i++)
			cout << error_h.H[i] << " ";
		cout << endl;
	}
	
	//saveOutput
	if (alg_saveOutput) {
	
		cout << " - saving to " << alg_saveOutputFile << " " ;
		if (strcmp(alg_saveOutputFormat,"sum")==0) {
			cout << "(summary mode)" << endl;
			ofstream fmatches(alg_saveOutputFile);
				if (!fmatches) {
					std::cerr << "Error : Unable to write '" << alg_saveOutputFile << "'" << std::endl;
					return;
				}
			for(int i=0;i<matches.size();i++) 
				fmatches << i << " " << matches[i].first.vertex->point() << " " << matches[i].second.vertex->point() << endl;
		}
		else if (strcmp(alg_saveOutputFormat,"idx")==0) {
			cout << "(index mode)" << endl;
			ofstream fmatches(alg_saveOutputFile);
			if (!fmatches) {
				std::cerr << "Error : Unable to write '" << alg_saveOutputFile << "'" << std::endl;
				return;
			}
			vector<int>indices;

			int local_index=0;
			for (Vertex_iterator vi = mesh1.p.vertices_begin(); vi!=mesh1.p.vertices_end(); vi++) {
				vi->id = local_index++;
			}
			
			local_index=0;
			for (Vertex_iterator vi = mesh2.p.vertices_begin(); vi!=mesh2.p.vertices_end(); vi++) {
				vi->id = local_index++;
				indices.push_back(-1);				
			}
			
			for(int i=0;i<matches.size();i++) 
				indices[matches[i].second.vertex->id] = matches[i].first.vertex->id;
			
			for(int i=0;i<indices.size();i++) 
				fmatches << indices[i] << endl;
		}
		else {
			cout << "(detailed mode)" << endl;
			ofstream fmatches(alg_saveOutputFile);
				if (!fmatches) {
					std::cerr << "Error : Unable to write '" << alg_saveOutputFile << "'" << std::endl;
					return;
				}


			saveDescriptorList(vec_vd1,fmatches);
			saveDescriptorList(vec_vd2,fmatches);
			//matches
			fmatches << matches.size() << endl;
			for(int i=0;i<matches.size();i++)
				fmatches << matches_index[i].first << " " << matches_index[i].second << endl;
		}

		if (evaluate_matches) { // save error histogram
			char alg_saveMatchesOutputFile[strlen(alg_saveOutputFile)+30];
			sprintf(alg_saveMatchesOutputFile,"%s.errors",alg_saveOutputFile);
			
			cout << " - saving error histogram to " << alg_saveMatchesOutputFile << endl;			
			ofstream ferrors(alg_saveMatchesOutputFile);			
			for(int i=0;i<bins;i++)
				ferrors << error_h.H[i] << " ";
					ferrors << endl;
		}
	}



/*
		// debug info
		for (int i=0;i<10;i++) {
			cout << vec_vd1[i] << endl;
			cout << vec_vd2[i] << endl;
		}
*/

}

