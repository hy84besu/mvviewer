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

#include "PointSet.h"

#define FILENAME_LENGTH 1024

/////////////////////////////////////////////////////////////////////////////////////////////////////
class VertexDescriptor {
	public:
		Vertex *vertex;
		vector<float> descriptor;

		VertexDescriptor(){vertex=NULL;}

		VertexDescriptor(Vertex &v) { vertex=&v;}

		void setVertex(Vertex &v) { vertex=&v;}

		void reset() {
			vertex=NULL;
			descriptor.clear();
		}
		bool isSet() {
			return vertex!=NULL;
		}
	
		void setDescriptor(const vector<float> &v_desc) { descriptor = v_desc; }
		void appendDescriptor(const vector<float> &v_desc) { 
			std::copy( v_desc.begin(), v_desc.end(), std::back_inserter( descriptor ) );
		}	

		void print() {
			for (int i=0;i<descriptor.size();i++)
				cout << descriptor[i] << " ";
		}

		// mode: 0 - L1 norm; 1 - L1 norm; 2 - L2 norm				   
		float norm(int mode=2) {
			float total=0.0f;
			for (int i=0;i<descriptor.size();i++) {
				if (mode==2) total+=descriptor[i]*descriptor[i];
			}
			if (mode==2) total=sqrt(total);
			return total;			
		}
				   
		// mode: 0 - dot product; 1 - euclidean distance
		float distanceTo(const VertexDescriptor &other, int mode=1) {
			float total=0.0f;
			for (int i=0;i<descriptor.size();i++) {
				if (mode==0) total+=descriptor[i]*other.descriptor[i];
				if (mode==1) total+=(descriptor[i]-other.descriptor[i])*(descriptor[i]-other.descriptor[i]);
			}
			if ((mode==1) && (total!=0.0f)) total=sqrt(total+0.00000000000001f);
			return total;
		}

		//~VertexDescriptor() { delete [] descriptor;}
};

std::ostream& operator<<(std::ostream& os, const VertexDescriptor& vd);

/////////////////////////////////////////////////////////////////////////////////////////////////////
class MeshMatching {
public:
    enum EQualityMeasureTypes {EQualColor=0, EQualMeanCurv=1, EQualGaussianCurv=2, EQualImported=3, EQualMixedColorMeanCurv=4,  EQualUndefined=5 };
    enum EScalespaceMethod {EScalespaceConvolution, EScalespaceSpectral};
    enum GTGenerator {EGTOneToOne, EGTGeodesic, EGTAutoDetect};
    enum SaveDescriptorFormat {SaveFormatPoints=0, SaveFormatDetDescriptors=1, SaveFormatDetectors=2, SaveFormatDescriptors=3, SaveFormatDetectorsBoolean=4};

    MeshMatching();
    ~MeshMatching();
    void setParams(const char* src_mesh_file, const char* dst_mesh_file, int curvature_no_rings, float feat_no_rings, bool treat_no_rings_as_surface_percentage, int feat_no_bins_centroid, int feat_no_bins_groups , int feat_no_bins_orientations , double feat_spatial_influence, bool feat_compute_all, double matching_2nd_ratio, const char * matches_dst_src_file, bool save_outputs, const char * save_output_filename, const char * save_output_format, const char * groundtruth, bool non_max_sup, bool scale_space, int no_scales, bool feat_usesDetScale, float corner_thresh, int feature_type, int detector_type, int detector_method, const char* detector_file, float det_thresh, const char * src_mesh_desc_file, const char * dst_mesh_desc_file, char op_mode, bool noise_data, float noise_sigma_colour, float noise_sigma_colour_shot, float noise_sigma_geometry, float noise_sigma_geometry_shot, float noise_sigma_geom_rotate, float noise_sigma_geom_scale, float noise_sigma_geom_local_scale, float noise_sigma_geom_sampling, float noise_sigma_geom_holes, float noise_sigma_geom_micro_holes, float noise_sigma_geom_topology);


    void run();


protected:

    void computeGradientStatistics(Mesh &mesh, int scale, float & fMean, float& fStddev);
    void computeFeatures(Mesh &mesh, vector<bool> &vec_isSelected);
    void computeScalespace(Mesh &mesh, EScalespaceMethod eMethod);
    void computeDescriptors(Mesh &mesh, vector<bool> &vec_isSelected, vector<VertexDescriptor> &vec_vd, bool fill_vec_vd);
    void computeColourFromDescriptors(Mesh &mesh, vector<VertexDescriptor> &vec_vd);
    void computeFeaturesDescriptors(Mesh &mesh, vector<bool> &vDet, vector<VertexDescriptor> &vDesc, bool bComputeAllDescriptors);
    void performMatching(vector<bool> &vDet1, vector<VertexDescriptor> &vDesc1, vector<bool> &vDet2, vector<VertexDescriptor> &vDesc2);
    void performEvaluation(vector<bool> &vDet1, vector<VertexDescriptor> &vDesc1, vector<bool> &vDet2, vector<VertexDescriptor> &vDesc2);
    void generateGroundtruthMatches(Mesh & m1, Mesh &m2, std::vector<int> &vMatch,  GTGenerator eGenMode, float fModeParam);
    void evaluateDetectorDescriptorWithGT(vector<bool> &vDet1, vector<VertexDescriptor> &vDesc1, vector<bool> &vDet2, vector<VertexDescriptor> &vDesc2, vector<int> &vMatches, bool bFullDescriptorMatch, Mesh &mesh, float geo_ball, float &det_rep, float &desc_norm);


    int getNoDescriptorRings(Mesh &m, float featNoRings, bool treatNoRingsAsSurfacePercentage);
    int getNoDetectorFeatures(Mesh &m);
    EQualityMeasureTypes convertToQualityMeasureType(int i);
    
    int selectRandomFeatures(int no_features, int total_vertices , vector<bool>& vDet);
    int  loadFeatureList(const char* filename, Mesh &mesh, vector<bool>& vDet);
    void loadFeatureDescriptorList(const char* filename, Mesh &mesh, vector<bool>& vDet, vector<VertexDescriptor>& vDesc);
    void loadGroundtruthMatches(const char* filename, vector<int>& vMatches);
    void saveFeatureDescriptorList(vector<bool> &vDet, vector<VertexDescriptor> &vec_vd, ostream &out, SaveDescriptorFormat saveFormat=SaveFormatDetDescriptors);
    void saveFeatureDescriptorList(vector<bool> &vDet, vector<VertexDescriptor> &vec_vd, char *fileTemplate, SaveDescriptorFormat saveFormat=SaveFormatDetDescriptors);
    void saveGroundtruthMatches(const char* filename, vector<int>& vMatches);	
    void saveGradientField(Mesh &mesh,int scale);
    const char* getSaveFormatExtension(SaveDescriptorFormat saveFormat);
    

    // members
	public:
		Mesh mesh1,mesh2;

		bool alg_featuresOnly;
		char alg_opMode; // 'F' - features only; 'M' - matching; 'E' - evaluation
	
		//separate
		int alg_curvNoRings;

		// detector
		enum EQualityMeasureTypes	alg_detType;
		int                         alg_detMethod;
        const char *                alg_detFile;
		float                       alg_detThresh;
		bool                        alg_nonMaxSup;
		float                       alg_cornerThresh;
		bool                        alg_scaleSpace;
		int                         alg_noScales;	
		int                         alg_noConvolves;
		int                         alg_convsPerScale;	
		double                      alg_convSigma;

		// feature
		EQualityMeasureTypes	alg_featType; 
		float					alg_featNoRings;
		bool					alg_treatNoRingsAsSurfacePercentage;
		int						alg_featNoBinsCentroid;
		int						alg_featNoBinsGroups;
		int						alg_featNoBinsOrientations;
		double					alg_featSpatialInfluence;
		bool					alg_featUsesDetScale;
		bool					alg_featComputeAll;
	
		//matching
		double					alg_matchingBestMatchesRatio;
		float					alg_geodesicBallGT;
	
		//noise 
		bool alg_noise;
		float alg_noiseSigmaColour;	
		float alg_noiseSigmaColourShot;	
		float alg_noiseSigmaGeometry;
		float alg_noiseSigmaGeometryShot;	
		float alg_noiseSigmaGeometryRotate;	
		float alg_noiseSigmaGeometryScale;	
		float alg_noiseSigmaGeometryLocalScale;		
		float alg_noiseSigmaGeometrySampling;	
		float alg_noiseSigmaGeometryHoles;
		float alg_noiseSigmaGeometryMicroHoles;	
		float alg_noiseSigmaGeometryTopology;	

		bool alg_saveOutput;
		char alg_saveOutputFile[FILENAME_LENGTH];
		char alg_saveOutputFormat[FILENAME_LENGTH];
		char alg_groundtruthFile[FILENAME_LENGTH];
		char alg_srcMeshFile[FILENAME_LENGTH];
		char alg_dstMeshFile[FILENAME_LENGTH];
		
		char alg_srcMeshDescFile[FILENAME_LENGTH];
		char alg_dstMeshDescFile[FILENAME_LENGTH];
		char alg_matchesDstSrcFile[FILENAME_LENGTH];	

		bool stopAlgPressed;
	protected:
		bool is_init;
		int cur_iter;
		typedef pair<VertexDescriptor,VertexDescriptor> vdpair;
		vector<vdpair> matches;
		vector<pair<int,int> > matches_index;
		map<Point,Point> matches_groundtruth;
};

#endif
