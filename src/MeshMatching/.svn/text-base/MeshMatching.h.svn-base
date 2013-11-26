/*
 *  MVAlg.h
 *  src
 *
 *  Created by Andrei Zaharescu on 15/12/06.
 *  Copyright 2006 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef MESH_MATCHING_H
#define MESH_MATCHING_H

#include "OpenGLContext.h"
#include "PointSet.h"
#include <QThread>
#include "MVData.h"


/////////////////////////////////////////////////////////////////////////////////////////////////////
class VertexDescriptor {
	public:
		Vertex *vertex;
		vector<float> descriptor;

		VertexDescriptor(){vertex=NULL;}

		VertexDescriptor(Vertex &v) { vertex=&v;}

		void setVertex(Vertex &v) { vertex=&v;}

		void setDescriptor(const vector<float> &v_desc) { descriptor = v_desc; }
		void appendDescriptor(const vector<float> &v_desc) { 
			std::copy( v_desc.begin(), v_desc.end(), std::back_inserter( descriptor ) );
		}	

		void print() {
			for (int i=0;i<descriptor.size();i++)
				cout << descriptor[i] << " ";
		}

		// mode: 0 - dot product; 1 - euclidean distance
		float distanceTo(const VertexDescriptor &other, int mode=0) {
			float total=0.0f;
			for (int i=0;i<descriptor.size();i++) {
				if (mode==0) total+=descriptor[i]*other.descriptor[i];
				if (mode==1) total+=(descriptor[i]-other.descriptor[i])*(descriptor[i]-other.descriptor[i]);
			}
			if (mode==1) total=sqrt(total);
			return total;
		}

		//~VertexDescriptor() { delete [] descriptor;}
};

std::ostream& operator<<(std::ostream& os, const VertexDescriptor& vd);

/////////////////////////////////////////////////////////////////////////////////////////////////////
class MeshMatching: public QThread {
		Q_OBJECT
	public:
		MeshMatching(MVData*);
		~MeshMatching();
		void setParams(const char* src_mesh_file, const char* dst_mesh_file, int feat_no_rings, bool treat_no_rings_as_surface_percentage, int feat_no_bins_centroid, int feat_no_bins_groups , int feat_no_bins_orientations , double feat_spatial_influence, double matching_2nd_ratio, bool save_outputs, const char * save_output_filename, const char * save_output_format, const char * groundtruth, bool non_max_sup, bool scale_space, float corner_thresh, int feature_type, int detector_type, float det_thresh, const char * src_mesh_desc_file, const char * dst_mesh_desc_file, bool features_only, bool noise_data, float noise_sigma_geometry, float noise_sigma_colour);

		static void saveDescriptorList(vector<VertexDescriptor> &vec_vd1, ostream &out, bool pointsOnly=false);
		void execute() { run();}
		void EndAlg();
		void importFeaturesFromFile(const char* filename, vector<Vertex>& vertex_storage, vector<VertexDescriptor>& output_matches);
	signals:
		void stepFinished();
	protected:
		virtual void run();
		void detectComputeFeatures(Mesh &mesh, vector<VertexDescriptor> &vec_vd);
//		float getGradient(Vertex &v, int mode);
		// members
	public:
		MVData *data;
		Mesh mesh1,mesh2;

		//separate
		int alg_curvNoRings;
		// feature
		int alg_featType; // 0 - color; 1 - mean curv; 2 - gaussian curv;
		int alg_featNoRings;
		int alg_featNoBinsCentroid;
		int alg_featNoBinsGroups;
		int alg_featNoBinsOrientations;
		double alg_featSpatialInfluence;
		// matching
		int alg_detType;
		float alg_detThresh;
		double alg_matchingBestMatchesRatio;
		bool alg_nonMaxSup;
		float alg_cornerThresh;
		bool alg_scaleSpace;

		bool alg_noise;
		float alg_noiseSigmaGeometry;
		float alg_noiseSigmaColour;	


		bool alg_treatNoRingsAsSurfacePercentage;
		bool alg_featuresOnly;
		bool alg_saveOutput;
		char alg_saveOutputFile[255];
		char alg_saveOutputFormat[255];
		char alg_groundtruthFile[255];
		char alg_srcMeshFile[255];
		char alg_dstMeshFile[255];
		
		char alg_srcMeshDescFile[255];
		char alg_dstMeshDescFile[255];

		bool stopAlgPressed;
	protected:
		bool is_init;
		int cur_iter;
		typedef pair<VertexDescriptor,VertexDescriptor> vdpair;
		vector<vdpair> matches;
		vector<pair<int,int> > matches_index;
		map<Point,Point> matches_groundtruth;

		vector<Vertex> loaded_vertices_mesh1, loaded_vertices_mesh2;
};

#endif
