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
#include "MeshNoiser.h"
#include "ColorMap.h"
#include <CGAL/Timer.h>


std::ostream& operator<<(std::ostream& os, const VertexDescriptor& vd) {
	os << "Vertex [" << vd.vertex->id << "] -> ";
	for (int d = 0; d < vd.descriptor.size(); ++d)
		os << ' ' << vd.descriptor[d];
	return os << ' ';
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor
MeshMatching::MeshMatching() 
{
	is_init = false;
	alg_saveOutput = false;
	alg_scaleSpace = true;
	alg_curvNoRings = 2;
	alg_detType=EQualColor;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
// Destructor
MeshMatching::~MeshMatching() 
{
}



//////////////////////////////////////////////////////////////////////////////////////////////////////
void MeshMatching::setParams(const char* src_mesh_file, const char* dst_mesh_file, int curvature_no_rings, float feat_no_rings, bool treat_no_rings_as_surface_percentage, int feat_no_bins_centroid, int feat_no_bins_groups , int feat_no_bins_orientations , double feat_spatial_influence, bool feat_compute_all, double matching_2nd_ratio, const char * matches_dst_src_file, bool save_outputs, const char * save_output_filename, const char * save_output_format, const char * groundtruth, bool non_max_sup, bool scale_space, int no_scales, bool feat_usesDetScale, float corner_thresh, int feature_type, int detector_type, int detector_method,  const char* detector_file, float det_thresh, const char * src_mesh_desc_file, const char * dst_mesh_desc_file, char op_mode, bool noise_data, 
			   float noise_sigma_colour, float noise_sigma_colour_shot, float noise_sigma_geometry, float noise_sigma_geometry_shot, float noise_sigma_geom_rotate, float noise_sigma_geom_scale, float noise_sigma_geom_local_scale, float noise_sigma_geom_sampling, float noise_sigma_geom_holes, float noise_sigma_geom_micro_holes, float noise_sigma_geom_topology) {

	bool printStats=true;
	is_init = true;

	alg_noScales = no_scales;
	alg_featUsesDetScale = feat_usesDetScale;
	
	if (printStats) cout << "Detection options:" << endl;		
	alg_detType = convertToQualityMeasureType(detector_type);

	if (printStats) cout << "- alg_detType = " << alg_detType << endl;
	alg_detMethod = detector_method;
	if (printStats) cout << "- alg_detMethod = " << alg_detMethod << endl;
    alg_detFile = detector_file;
	if (printStats) cout << "- alg_detFile = " << alg_detFile << endl;
    
	alg_detThresh = det_thresh;
	if (printStats) cout << "- alg_detThresh = " << alg_detThresh << endl;
	alg_nonMaxSup = non_max_sup;
	if (printStats) cout << "- alg_nonMaxSup = " << alg_nonMaxSup << endl;
	alg_scaleSpace = scale_space;
	if (printStats) cout << "- alg_scaleSpace = " << alg_scaleSpace << endl;
	if (printStats) cout << "- alg_noScales = " << alg_noScales << endl;	
	alg_cornerThresh = corner_thresh;
	if (printStats) cout << "- alg_cornerThresh = " << alg_cornerThresh << endl;

	
	if (printStats) cout << "Descriptor options:" << endl;
	alg_featType = convertToQualityMeasureType(feature_type);
	if (printStats) cout << "- alg_featType = " << alg_featType << endl;
	if (printStats) cout << "- alg_featUsesDetScale = " << alg_featUsesDetScale << endl;	
	alg_featNoRings = feat_no_rings;
	if (printStats) cout << "- alg_featNoRings = " << alg_featNoRings << endl;	
	alg_treatNoRingsAsSurfacePercentage=treat_no_rings_as_surface_percentage;
	if (printStats) cout << "- treat no rings as surface percentage = " << treat_no_rings_as_surface_percentage << endl;
	alg_featNoBinsCentroid = feat_no_bins_centroid;
	if (printStats) cout << "- alg_featNoBinsCentroid = " << alg_featNoBinsCentroid << endl;	
	alg_featNoBinsGroups = feat_no_bins_groups;
	if (printStats) cout << "- alg_featNoBinsGroups = " << alg_featNoBinsGroups << endl;
	alg_featNoBinsOrientations = feat_no_bins_orientations;
	if (printStats) cout << "- alg_featNoBinsOrientations = " << alg_featNoBinsOrientations << endl;
	alg_featSpatialInfluence = feat_spatial_influence;
	if (printStats) cout << "- alg_featSpatialInfluence = " << alg_featSpatialInfluence << endl;
	alg_featComputeAll = feat_compute_all;
	if (printStats) cout << "- alg_featComputeAll = " << alg_featComputeAll << endl;

	if (printStats) cout << "Matching Options:" << endl;	
	alg_matchingBestMatchesRatio = matching_2nd_ratio;
	if (printStats) cout << "- alg_matchingBestMatchesRatio = " << alg_matchingBestMatchesRatio << endl;

	
	if (printStats) cout << "Noise Options:" << endl;	
	alg_noise=noise_data;
	if (printStats) cout << " alg_noise = " << alg_noise << endl;
	if (alg_noise)
	{
		alg_noiseSigmaColour=noise_sigma_colour;
		if (printStats) cout << " alg_noiseSigmaColour = " << alg_noiseSigmaColour << endl;
		alg_noiseSigmaColourShot=noise_sigma_colour_shot;
		if (printStats) cout << " alg_noiseSigmaColourShot = " << alg_noiseSigmaColourShot << endl;
		
		alg_noiseSigmaGeometry=noise_sigma_geometry;
		if (printStats) cout << " alg_noiseSigmaGeometry = " << alg_noiseSigmaGeometry << endl;
		alg_noiseSigmaGeometryShot=noise_sigma_geometry_shot;
		if (printStats) cout << " alg_noiseSigmaGeometryShot = " << alg_noiseSigmaGeometryShot << endl;
		
		alg_noiseSigmaGeometryRotate=noise_sigma_geom_rotate;
		if (printStats) cout << " alg_noiseSigmaGeometryRotate = " << alg_noiseSigmaGeometryRotate << endl;
		alg_noiseSigmaGeometryScale=noise_sigma_geom_scale;
		if (printStats) cout << " alg_noiseSigmaGeometryScale = " << alg_noiseSigmaGeometryScale << endl;
		alg_noiseSigmaGeometryLocalScale=noise_sigma_geom_local_scale;
		if (printStats) cout << " alg_noiseSigmaGeometryLocalScale = " << alg_noiseSigmaGeometryLocalScale << endl;
		alg_noiseSigmaGeometrySampling=noise_sigma_geom_sampling;
		if (printStats) cout << " alg_noiseSigmaGeometrySampling = " << alg_noiseSigmaGeometrySampling << endl;
		alg_noiseSigmaGeometryHoles=noise_sigma_geom_holes;
		if (printStats) cout << " alg_noiseSigmaGeometryHoles = " << alg_noiseSigmaGeometryHoles << endl;
		alg_noiseSigmaGeometryMicroHoles=noise_sigma_geom_micro_holes;
		if (printStats) cout << " alg_noiseSigmaGeometryMicroHoles = " << alg_noiseSigmaGeometryMicroHoles << endl;
		alg_noiseSigmaGeometryTopology=noise_sigma_geom_topology;
		if (printStats) cout << " alg_noiseSigmaGeometryTopology = " << alg_noiseSigmaGeometryTopology << endl;
	}
	
	
	if (printStats) cout << "Other Options:" << endl;	
    alg_curvNoRings   =curvature_no_rings;
	if (printStats) cout << " alg_curvatureNoRings = " << alg_curvNoRings << endl;    
	alg_saveOutput = save_outputs;
	if (printStats) cout << " alg_saveOutput = " << alg_saveOutput << endl;
	alg_opMode = op_mode;
	if (printStats) cout << " alg_opMode = " << alg_opMode << endl;
	
	strcpy(alg_srcMeshFile,src_mesh_file);
	strcpy(alg_dstMeshFile,dst_mesh_file);
	strcpy(alg_srcMeshDescFile,src_mesh_desc_file);
	strcpy(alg_dstMeshDescFile,dst_mesh_desc_file);
	strcpy(alg_saveOutputFile,save_output_filename);
	strcpy(alg_saveOutputFormat,save_output_format);
	strcpy(alg_groundtruthFile,groundtruth);
	
	strcpy(alg_matchesDstSrcFile,matches_dst_src_file);	

}

//////////////////////////////////////////////////////////////////////////////////////////////////////
int MeshMatching::selectRandomFeatures(int no_features, int total_vertices , vector<bool>& vDet)
{
    vDet.clear();
    int iTotal = 0;
    float fRatio = 1.0f*no_features / total_vertices;
    for(int i=0;i < total_vertices; i++)
    {
        if (rand() <= fRatio*RAND_MAX)
        {
            vDet.push_back(true);
            iTotal++;
        }
        else
            vDet.push_back(false);
    }
    return iTotal;
}

#pragma mark -
#pragma mark Load/Save

//////////////////////////////////////////////////////////////////////////////////////////////////////
int MeshMatching::loadFeatureList(const char* filename, Mesh &mesh, vector<bool>& vDet) 
{    	
	try
	{
		ifstream ifs(filename);
		if (!ifs.good())
		{
			cout << "* ERROR Importing features from "<<filename << " - could not open file" << endl;
			return 0;
		}
		
		int iTotal=0;	
        for(int i=0;i<mesh.p.size_of_vertices();i++) 
        {
            int value;
            ifs >> value;
            if (value>0) 
                iTotal++;
            vDet.push_back(value>0);
        }
		return iTotal;
	}
	catch (ios_base::failure &f) {
		cout << "Critical failure while... " << f.what() << '\n';
	}

	if (mesh.p.size_of_vertices()!=vDet.size())
	{
		cout << "The number of vertices from the file " << vDet.size() << " is different from the number of vertices of the mesh" << mesh.p.size_of_vertices() << endl;
	}
	vDet.clear();
	return 0;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
void MeshMatching::loadFeatureDescriptorList(const char* filename, Mesh &mesh, vector<bool>& vDet, vector<VertexDescriptor>& vDesc) {

	VertexDescriptor vd;	
	Vertex *v;
	vector<float> desc;
	float tmpF;
	int id;
	float a;
//    float b,c;
	int no_features, descriptor_size, elems_per_row;
	bool fillVector = (alg_opMode=='E');
	
	try
	{
		ifstream ifs(filename);
		if (!ifs.good())
		{
			cout << "* ERROR Importing features / descriptors from "<<filename << " - could not open file" << endl;			
			return;
		}
		
		ifs >> no_features;
		ifs >> descriptor_size;
		ifs >> elems_per_row;

		vd.reset();	
		vDet.clear();
		vDesc.clear();
		for(int i=0;i<mesh.p.size_of_vertices();i++) 
		{
			if (fillVector) vDesc.push_back(vd);		// create the vec_vd vector
			vDet.push_back(false);
			
		}
		
		cout << "* Importing " << no_features << " features & descriptors from "<<filename << endl;
		
		for(int i=0;i<no_features;i++) {

			// get the id
			ifs >> id;
			v = mesh.getVertexById(id);
			//cout << id << " ";
			// get the descriptor
			desc.clear();
			for(int j=0;j<descriptor_size;j++) 
			{
				ifs>>tmpF;
				desc.push_back(tmpF);
			}
		
			// ignore other element in the current row
			for(int j=0;j<elems_per_row-1-descriptor_size; j++)
				ifs >> a;
			
//			ifs >> a >> b >> c; // the point
//			ifs >> a >> b >> c; // the normal 
//			ifs >> a >> b >> c; // the colour

			// set the data
			vd.setVertex(*v);
			vd.setDescriptor(desc);
			vDet[id] = true;
			if (fillVector)
				vDesc[id] = vd;
			else
				vDesc.push_back(vd);
			
		} 
		
	}
	catch (ios_base::failure &f) {
		cout << "Critical failure while... " << f.what() << '\n';
	}
	
/*	int totalDet=0;
	for(int i=0;i<vDet.size();i++)
		if (vDet[i]) totalDet++;
	cout << "* Total detections is  " << totalDet << " in  "<<filename << endl;
*/	
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
void MeshMatching::saveFeatureDescriptorList(vector<bool> &vDet, vector<VertexDescriptor> &vDesc, char *fileTemplate, SaveDescriptorFormat saveFormat)
{
	const char *fileExt = getSaveFormatExtension(saveFormat);
	char filename[strlen(fileTemplate)+strlen(fileExt) + 2];
	sprintf(filename,"%s.%s",fileTemplate,fileExt);
	
	ofstream out(filename);
	if (!out) {
		std::cerr << "Error : Unable to write '" << filename << "'" << std::endl;
		return;
	}
	saveFeatureDescriptorList(vDet,vDesc,out,saveFormat);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
void MeshMatching::saveFeatureDescriptorList(vector<bool> &vDet, vector<VertexDescriptor> &vec_vd, ostream &out, SaveDescriptorFormat saveFormat) 
{
    if (saveFormat == SaveFormatDetectorsBoolean) 
    {
        for(int i=0;i<vDet.size(); i++)
        if (vDet[i])
            out << "1" << endl;
        else 
            out << "0" << endl;
        return;
    }

    
	int descriptor_size = 0;
	if (vec_vd.size()>0) descriptor_size = vec_vd[0].descriptor.size() ;
	int elems_per_row = 1 + descriptor_size;
	if (saveFormat == SaveFormatDetDescriptors)
	{
		out << vec_vd.size()  << " " << descriptor_size << " " << elems_per_row << endl;
	}
	

	for(int i=0;i<vec_vd.size();i++) {
		if (saveFormat == SaveFormatPoints) 
		{
			if (vec_vd[i].isSet()) 			
				out << vec_vd[i].vertex->point() << endl;
		}
		else if (saveFormat == SaveFormatDetDescriptors) 
		{
			if (vec_vd[i].isSet()) {
				// id 
				out << vec_vd[i].vertex->id << " ";				
				// descriptor
				for(int j=0;j<vec_vd[i].descriptor.size();j++)
					out << vec_vd[i].descriptor[j] << " ";
/*				//point
				out << vec_vd[i].vertex->point() << " ";
				//normal
				out << vec_vd[i].vertex->normal();
				//colour
				float hsv[3];
				float* color = vec_vd[i].vertex->color;
				//ColorMap::RGBtoHSV(color[0],color[1],color[2],hsv);
				out << color[0] << " " << color[1] << " " << color[2] << " ";
*/
				out << endl;				
			}
			else {
//				out << i << " " << " -1" << endl;
			}
		}
		else if (saveFormat == SaveFormatDescriptors) 
		{
			if (vec_vd[i].isSet()) {
				// descriptor
				for(int j=0;j<vec_vd[i].descriptor.size();j++)
					out << vec_vd[i].descriptor[j] << " ";
				out << endl;				
			}
		}
		else if (saveFormat == SaveFormatDetectors) // for matlab
		{
			if (vec_vd[i].isSet()) {
				// get a facet
				Facet_handle f = vec_vd[i].vertex->halfedge()->facet();
				HF_circulator j = f->facet_begin();
				// circulate around the vertices and see the position of the vertex - needed for baricentric coords
				CGAL_assertion ( CGAL::circulator_size ( j ) == 3 );
				int facetPos = 0;
				do
				{
					if (j->vertex()->id ==vec_vd[i].vertex->id) break;
					facetPos++;
				}
				while ( ++j != f->facet_begin() );
					
				out << vec_vd[i].vertex->halfedge()->facet()->id << " ";
				for(int k=0;k<3;k++) 
				{
					if (k==facetPos) out << "1 ";
					else out << "0 ";
				}
				out << endl;		
			}
		}
	}
}



//////////////////////////////////////////////////////////////////////////////////////////////////////
void MeshMatching::loadGroundtruthMatches(const char* filename, vector<int>& vMatches) {
	
	ifstream ifs(filename);
	if (!ifs.good())
	{
		cout << "* ERROR importing groundtruth matches from "<<filename <<" - could not open file" << endl;	
		return;
	}
	int no_matches;
	ifs >> no_matches;

	cout << "* Importing " << no_matches << " groundtruth matches from "<<filename << endl;
	vMatches.clear();
	for(int i=0;i<no_matches;i++) 
	{
		int id;
		ifs >> id;
		vMatches.push_back(id);
	} 
	
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
void MeshMatching::saveGroundtruthMatches(const char* filename, vector<int>& vMatches) {
	
	ofstream ofs(filename);
	int no_matches = vMatches.size();
	ofs << no_matches << endl;
	
	cout << "* Exporting " << no_matches << " groundtruth matches to "<<filename << endl;
	for(int i=0;i<no_matches;i++) 
	{
		ofs << vMatches[i] << endl;
	} 	
}




void MeshMatching::saveGradientField(Mesh &mesh,int scale)
{
	// save gradient info
	char prefix[1000] = "./"; ///Users/andreiz/Projects/INRIA/TransforMesh
	char scalarFunctionFile[1000];
	FILE *f2;
	
	// export mesh scalar function to file
	sprintf(scalarFunctionFile,"%s%s",prefix, "mesh_scalar_function_gradient.txt");
	f2=fopen(scalarFunctionFile,"w");
	for ( Vertex_iterator vi = mesh.p.vertices_begin(); vi!=mesh.p.vertices_end(); vi++ )
	{
		fprintf(f2,"%f ",vi->values[2*scale + 1]);
	}
	fclose(f2);
	
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
void MeshMatching::computeGradientStatistics(Mesh &mesh, int scale, float& fMean, float &fStdev)
{
	fMean=0.0f;
	fStdev=0.0f;
	int iCount=0;
	
	// compute mean
	for (Vertex_iterator vi = mesh.p.vertices_begin(); vi!=mesh.p.vertices_end(); vi++) 
		if (!vi->flag[0])  // should be included
		{
			fMean+=abs(vi->values[2*scale+1]);
			iCount++;
		}
	
	if (iCount==0) return;
	fMean/=iCount;

	// std dev
	for (Vertex_iterator vi = mesh.p.vertices_begin(); vi!=mesh.p.vertices_end(); vi++) 
		if (!vi->flag[0])  // should be included
		{
			float fTmp = fMean - abs(vi->values[2*scale+1]); 
			fStdev += fTmp*fTmp;
		}
	fStdev=sqrt(fStdev/iCount);
}

#pragma mark -
#pragma mark Helper

//////////////////////////////////////////////////////////////////////////////////////////////////////
int MeshMatching::getNoDescriptorRings(Mesh &m, float featNoRings, bool treatRingsAsPercentage) {
	if (treatRingsAsPercentage)
		return  max(2,(int)round(m.getSurfacePercentageRadius(featNoRings)/m.edge_avg));
	else
		return (int)round(featNoRings);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
int MeshMatching::getNoDetectorFeatures(Mesh &mesh)
{
	if (alg_detThresh<=1.0f)
		return (int)round(alg_detThresh*mesh.p.size_of_vertices());
	else
		return alg_detThresh; 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
const char* MeshMatching::getSaveFormatExtension(SaveDescriptorFormat saveFormat)
{
	if (saveFormat==SaveFormatPoints) return "points";
	if (saveFormat==SaveFormatDetDescriptors) return "det_desc";	
	if (saveFormat==SaveFormatDetectors) return "feat";	
	if (saveFormat==SaveFormatDetectorsBoolean) return "featb";    
	if (saveFormat==SaveFormatDescriptors) return "desc";
	return "unknown";
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
MeshMatching::EQualityMeasureTypes MeshMatching::convertToQualityMeasureType(int i)
{
	if (i==0) return EQualColor;
	if (i==1) return EQualMeanCurv;
	if (i==2) return EQualGaussianCurv;
	if (i==3) return EQualImported;
	if (i==4) return EQualMixedColorMeanCurv;
	return EQualUndefined;
}

#pragma mark -
#pragma mark Computation

//////////////////////////////////////////////////////////////////////////////////////////////////////
void MeshMatching::computeScalespace(Mesh &mesh, EScalespaceMethod eMethod)
{
	
	for (Vertex_iterator vi = mesh.p.vertices_begin(); vi!=mesh.p.vertices_end(); vi++) 
	{
        //		cout << "q=" << vi->quality() << " mc=" << vi->mean_curvature() << " " ;
		vi->values.clear();
	}
    
	
	if (eMethod==EScalespaceConvolution)
	{
		//perform the convolutions - the qualities and (difference btw adjacent qualities) are stored in values as alternating
		for(int i=0;i<alg_noScales;i++) {
			for(int j=0;j<alg_convsPerScale;j++) {
				cout << "\r - building scale (convolutions): " << i + 1 << "/" << alg_noScales << " - octave " << j + 1 << "/" << alg_convsPerScale ; cout.flush();
				for(int ii=1;ii<(1<<i);ii++) mesh.convolve(alg_convSigma,false,false);
				mesh.convolve(alg_convSigma,true,false);	
				//mesh.convolve(pow(2.0,2*i/2.0)*alg_convSigma,true,false);
			}
		}
		
		cout << endl;	
	}
	else if (eMethod == EScalespaceSpectral)
	{
		cout << " - building scale (spectral decomposition - MATLAB): " << alg_noScales << " scales; " << alg_convsPerScale << " per octave" << endl;
		
		char prefix[1000] = "/Users/andreiz/Projects/INRIA/TransforMesh/";
		char scalarFunctionFile[1000];
		char buff[1000];
		char tmpFile[1000] = "bla.txt";
		FILE *f;
        
		// export mesh scalar function to file
		sprintf(scalarFunctionFile,"%s%s",prefix, "mesh_scalar_function.txt");
		f=fopen(scalarFunctionFile,"w");
		for ( Vertex_iterator vi = mesh.p.vertices_begin(); vi!=mesh.p.vertices_end(); vi++ )
		{
			fprintf(f,"%f ",vi->quality());
		}
		fclose(f);
		
		// prepare script
		sprintf(buff,"%s./matlab/scripts/invoke_matlab_scalespace.sh %s%s %s %s%s %d %d %f",prefix, prefix, mesh.loaded_filename, scalarFunctionFile, prefix, tmpFile, alg_noScales,alg_convsPerScale,alg_convSigma);
		
		// invoke script
		system(buff);
        
		// read generated output
		sprintf(buff,"%s/%s",prefix,tmpFile);
		f=fopen(buff,"r");
        
		for(int i=0; i<alg_noScales*alg_convsPerScale*2; i++)
		{
			for ( Vertex_iterator vi = mesh.p.vertices_begin(); vi!=mesh.p.vertices_end(); vi++ )
			{
				float tmpVal=0.0f;
				if (f && (!feof(f))) fscanf(f,"%f",&tmpVal);
				vi->values.push_back(tmpVal);
			}
		}
		
		if (!f) cout << "- WARNING: MATLAB output file cannot be found or it is incomplete" <<endl;
        
		fclose(f);
	}
	
	
	// saveGradientField(mesh,0);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
void MeshMatching::computeFeatures(Mesh &mesh, vector<bool> &is_chosen_elems) {
	
	int bins=1000;
	int v_id=0;
	float percentile=alg_detThresh;
	
	float quality_min=std::numeric_limits<float>::max();
	float quality_max=std::numeric_limits<float>::min();
	vector<float> lower_bound;

	alg_noConvolves=1;
	alg_convSigma=1.25;
	alg_convsPerScale = 6;

	if (alg_detType==EQualColor) mesh.setMeshQuality(Mesh::Qual_Color);
	if (alg_detType==EQualMeanCurv) mesh.setMeshQuality(Mesh::Qual_Mean_Curv);	
	if (alg_detType==EQualGaussianCurv) mesh.setMeshQuality(Mesh::Qual_Gaussian_Curv);		
	if (alg_detType==EQualImported) mesh.setMeshQuality(Mesh::Qual_Imported);			
	
	cout << "* Detection" << endl;

    if ((alg_detMethod==2) || (alg_detMethod==3))
    {
//        for(int i=0;i<alg_curvNoRings;i++)
//           mesh.convolve(alg_convSigma,false,false);
    }
    if (alg_detMethod==2)
	{
        int iCount = selectRandomFeatures(alg_detThresh, mesh.p.size_of_vertices(), is_chosen_elems);
        cout << " - Generated  " << iCount << " random features" <<endl;
        alg_featUsesDetScale = false;
        alg_scaleSpace = 0;
        return;
    }
	else if (alg_detMethod==3)
	{
        int iCount = loadFeatureList(alg_detFile, mesh, is_chosen_elems);
        cout << " - Imported " << iCount << " features from file" <<endl;	
        alg_featUsesDetScale = false;
        alg_scaleSpace = 0;
        return;
    }

	vector<float> qual_elems;

	
	if (alg_scaleSpace) {
		alg_convSigma = pow(2.0,1.0/(alg_convsPerScale-3));//1.4142*
		alg_noConvolves=alg_convsPerScale*alg_noScales;
	}

	//perform the scalespace - the results are stored in the values vector within each vertex as: F(1), F(1)-F(0), F(2), F(2) - F(1), etc..
	if (alg_detMethod==0)
		computeScalespace(mesh,EScalespaceConvolution);
	else if (alg_detMethod==1)
		computeScalespace(mesh,EScalespaceSpectral);	
	
	cout << " - " << alg_detThresh << "th percentile = " << getNoDetectorFeatures(mesh)	<< " vertices" <<endl;	
	
	//reset the flags
	for (Vertex_iterator vi = mesh.p.vertices_begin(); vi!=mesh.p.vertices_end(); vi++) {
		vi->flag[0]=false; // border
		vi->flag[1]=false; //extrema
		vi->flag[2]=true; //corner
		vi->flag[3]=false; //threshold		
	}
	
	
	//see if it is an open surface, then mark the outside borders not to be included
	if (mesh.p.size_of_border_edges() > 0) 
	{
		//int noRings = max(1,(int)floor(1.0f*getNoDescriptorRings(mesh,alg_featNoRings,alg_treatNoRingsAsSurfacePercentage)));
		int noRings = 0;
		cout << " - detected an open surface => outer " << noRings << " ring regions will be ignored from detections (";
		// get list of border vertices
		vector<Vertex*> vertices;

		// select the vertices with 0 degree, if any
		for (Vertex_iterator vi = mesh.p.vertices_begin(); vi!= mesh.p.vertices_end(); vi++) 
			if (vi->vertex_degree()==0) vertices.push_back(&(*vi));
		
		for (Halfedge_iterator hi = mesh.p.border_halfedges_begin(); hi!= mesh.p.halfedges_end(); hi++) 
			vertices.push_back(&(*hi->vertex()));

		// get the list of bordering rings
		vector<pair<Vertex*,int> > neighs = Mesh::getRingNeighbourhood(vertices,noRings,true);				
		// set the found bordering rings
		for (int j=0;j<neighs.size();j++) 
			neighs[j].first->flag[0] = true; //border
		cout << "border=" << vertices.size() << " total=" << neighs.size() << ")" << endl;
	}
	else
		cout << " - detected a closed surface => all vertices are considered" << endl;

	for (Vertex_iterator vi = mesh.p.vertices_begin(); vi!=mesh.p.vertices_end(); vi++) 
		if (vi->vertex_degree() == 0)
			vi->flag[0]=true; // empty vertex
	/*
	 cout << "Sample: ";
	 Vertex &vvv=*mesh.p.vertices_begin();
	 for(int i=0;i<no_convolutions;i++) 
		 cout << vvv.values[i] << " ";
	 cout << endl;
	 cout << "No. of values << " << vvv.values.size() << endl;
*/
	
	std::vector<int> scale_stats;
	int totalScaleStats=0;
	
	float fGradMean, fGradStddev;
	
	cout << " - detection (0): detections per scale: ";
	for(int i=0;i<alg_noConvolves;i++) {
		scale_stats.push_back(0);

		//compute statistics (mean, stddev) on the gradient at each scale
		computeGradientStatistics(mesh,i,fGradMean,fGradStddev);
		
		for (Vertex_iterator vi = mesh.p.vertices_begin(); vi!=mesh.p.vertices_end(); vi++) {
			if (vi->flag[0]==false) {
				
				if (vi->flag[1]) continue; // not a border vertex and not selected yet 
				
				bool is_chosen=true;
				
				// if not away enough from the mean, ignore this gradient - not strong enough
//				if (abs(vi->values[2*i+1])<2.0f*fGradStddev) continue;
				
				int iScale = 1;
				//if (alg_scaleSpace && ((i<alg_convsPerScale*iScale) || (i>=alg_convsPerScale*(iScale+1)))) is_chosen=false;
//				if (alg_scaleSpace && (i<alg_convsPerScale*iScale) ) is_chosen=false;
				if (alg_nonMaxSup) 
				{
					vector<pair<Vertex*,int> > neighs = Mesh::getRingNeighbourhood(*vi,1,true);
					float refVal=vi->values[2*i+1];
					float minVal=refVal;
					float maxVal=refVal;
					// go through the list of neighbours
					for (int j=0;j<neighs.size();j++) 
					{
						if (j!=0) // neighbours on the same level
						{
							float tmpVal = neighs[j].first->values[2*i+1];
							minVal = std::min(tmpVal,minVal);
							maxVal = std::max(tmpVal,maxVal);
						}
							
						// neighbours on the previous level	
						if (i>0)
						{
							float tmpVal = neighs[j].first->values[2*i-1];
							minVal = std::min(tmpVal,minVal);
							maxVal = std::max(tmpVal,maxVal);
						}
						// neighbours on the next level
						if (i<alg_noConvolves-1)

						{
							float tmpVal = neighs[j].first->values[2*i+3];
							minVal = std::min(tmpVal,minVal);
							maxVal = std::max(tmpVal,maxVal);
						}
						if ((minVal!=refVal) && (maxVal!=refVal))
						{
							is_chosen = false; break;
						}
					}
					
					// the area is flat
					if ((is_chosen) && ((minVal==maxVal) && (minVal==refVal))) is_chosen = false;
				}
				
				if (is_chosen) 
				{
					if (vi->flag[1])  // if it was chosen before, remove it
					{ 
						scale_stats[vi->values[vi->values.size()-1]]--;						
						totalScaleStats--;
						vi->values.pop_back();
						vi->values.pop_back();
					}
					vi->flag[1]=true; // chosen
					vi->values.push_back(vi->values[2*i+1]); //the value
					vi->values.push_back(i); // the scale
					//stats
					scale_stats[scale_stats.size()-1]++;
				}
			}
		}
		cout << scale_stats[scale_stats.size()-1] << " ";
		totalScaleStats+=scale_stats[scale_stats.size()-1];
	}
	cout << endl;

	//find the thresholds for each individual scale
	float detThreshScale = min(1.0f,1.0f*getNoDetectorFeatures(mesh)/totalScaleStats);
	std::vector<int> vElemsPerScale;
	std::vector<int> vChosenElemsPerScale;		
	
	// compute the thresholds per scale
	int totalThresholds=0;
	for(int i=0;i<alg_noConvolves;i++) {
		vElemsPerScale.push_back((int)floor(detThreshScale*scale_stats[i]));
		vChosenElemsPerScale.push_back(0);
		totalThresholds+=vElemsPerScale[i];
	}
	
	int diffLeft = getNoDetectorFeatures(mesh) - totalThresholds;
	// make sure that we do not miss detections due to round-errors; fill-in where available
	for(int i=alg_noConvolves-1;i>=0;i--) 
	{
		if (diffLeft<=0) break;
		if (scale_stats[i] - vElemsPerScale[i] > 0)
		{
			int toAdd=std::min(diffLeft,scale_stats[i] - vElemsPerScale[i]);
			vElemsPerScale[i]+=toAdd;
			diffLeft-=toAdd;
		}
	}

	totalThresholds=0;
	for(int i=0;i<alg_noConvolves;i++) {
		totalThresholds+=vElemsPerScale[i];
	}
	
	//cout << "getThreshScale=" << detThreshScale  << endl;
	for(int i=0;i<alg_noConvolves;i++) {
		qual_elems.clear();
		// push the elements into qual_elems
		for (Vertex_iterator vi = mesh.p.vertices_begin(); vi!=mesh.p.vertices_end(); vi++) 
			if ((vi->flag[1]==true) && (vi->values[2*alg_noConvolves+1]==i)) // only if extrema at the right scale
				qual_elems.push_back(abs(vi->values[2*alg_noConvolves]));

		// find the threshold
		int nth_elem = (int)qual_elems.size() - vElemsPerScale[i]; 
		if (nth_elem<0) nth_elem=0;

		std::nth_element(qual_elems.begin(),qual_elems.begin()+nth_elem,qual_elems.end()); 

	/*
		cout << "qual_elems.size()=" << qual_elems.size() << endl;
		cout << "nth_elem=" << nth_elem << endl;
		cout << "std::nth_element ==> elem[" << nth_elem << "]=" << qual_elems[nth_elem] << " "  << endl;
		std::sort(qual_elems.begin(), qual_elems.end());
		cout << "------------" << endl;
		std::copy(qual_elems.begin(), qual_elems.end(), ostream_iterator<float>(cout, " "));
		cout << "std::sort ==> elem[" << nth_elem << "]=" << qual_elems[nth_elem] << " "  << endl;
	*/
	
		if (qual_elems.size() > 0)
			lower_bound.push_back(qual_elems[nth_elem]);
		else
			lower_bound.push_back(std::numeric_limits<float>::max());
		//cout  << "nth element for n=" << nth_elem << " and qual_elems.size=" << qual_elems.size() << " bound=" << lower_bound[i] << endl;
	}
	
	
	// threshold responses
	// a) first traverse and look for strict inequality
	for(int i=0;i<alg_noConvolves;i++) 
		for (Vertex_iterator vi = mesh.p.vertices_begin(); vi!=mesh.p.vertices_end(); vi++) 
			if ((vi->flag[1]==true) && (vi->values[2*alg_noConvolves+1]==i) && (vChosenElemsPerScale[i]<vElemsPerScale[i]))  // only at the right scale
			{
				vi->flag[3] = lower_bound[i] < abs(vi->values[2*alg_noConvolves]); //treshold
				if (vi->flag[3]) vChosenElemsPerScale[i]++;
				//if (vi->flag[3]) cout << "scale=" << i << " response=" << vi->values[2*alg_noConvolves] << endl;
			}
	// b) traverse and look for equality and choose up to max number
	for(int i=0;i<alg_noConvolves;i++) 
		for (Vertex_iterator vi = mesh.p.vertices_begin(); vi!=mesh.p.vertices_end(); vi++) 
			if ((vi->flag[3]==false) && (vi->flag[1]==true) && (vi->values[2*alg_noConvolves+1]==i) && (vChosenElemsPerScale[i]<vElemsPerScale[i]))  // only at the right scale
			{
				vi->flag[3] = (lower_bound[i] == abs(vi->values[2*alg_noConvolves])); //treshold
				if (vi->flag[3]) vChosenElemsPerScale[i]++;
				//if (vi->flag[3]) cout << "scale=" << i << " response=" << vi->values[2*alg_noConvolves] << endl;
			}

	
	// eliminate the corner responses
	if (alg_cornerThresh>0)
	{
		for(int i=0;i<alg_noConvolves;i++) {
			// set the quality for the scale
			for (Vertex_iterator vi = mesh.p.vertices_begin(); vi!=mesh.p.vertices_end(); vi++) 
				vi->quality() = vi->values[2*i] + 1; // 2*i - scalar value; (2*i+1) - dog
			// compute the quality derivatives
			mesh.setMeshQuality(Mesh::Qual_Quality_Deriv);		
			
			for (Vertex_iterator vi = mesh.p.vertices_begin(); vi!=mesh.p.vertices_end(); vi++) {
				if ((vi->flag[3]) && (vi->values[2*alg_noConvolves+1]==i)) { // only at the right scale
					if (lower_bound[i] <= abs(vi->values[2*alg_noConvolves])) {
						vi->flag[2] = mesh.isCorner(*vi,alg_cornerThresh); // needs qual_vec set 
					}
						
				}
			}
		}
	}

	//set the final list of chosen vertices
	v_id=0;		
	
	int counter_extrema=0;
	int counter_threshold=0;
	int counter_corner=0;
	
	for (Vertex_iterator vi = mesh.p.vertices_begin(); vi!=mesh.p.vertices_end(); vi++, v_id++) {
		// extrema
		if (vi->flag[1]) counter_extrema++;
		else continue;
		
		// threshold
		vi->flag[1] = vi->flag[3];		
		if (vi->flag[1]) counter_threshold++;
		else continue;

		// corner
		vi->flag[1] = vi->flag[2]; 		
		if (vi->flag[1]) counter_corner++;
	}

	//set the output
	is_chosen_elems.clear();
	for (Vertex_iterator vi = mesh.p.vertices_begin(); vi!=mesh.p.vertices_end(); vi++) {
		is_chosen_elems.push_back(vi->flag[1]);		
		vi->flag[2] = true; // reset the visibility flag so that edge_avg can be properly computed
	}
//	mesh.computeMeshStats();	
	cout << " - detection (1): extrema of gradient; points kept=" <<  counter_extrema << endl;
	cout << " - detection (2): magnitude thresholding; points kept=" << counter_threshold << endl;
	if (alg_cornerThresh>0)
		cout << " - detection (3): corner thresholding; points kept = " << counter_corner  << endl;
	else
		cout << " - detection (3): no corner thresholding; points kept = " << counter_corner  << endl;
	
	
}


// fill_vector - if set, then an entry is added for each mesh vertex, such that indexing is easy
//////////////////////////////////////////////////////////////////////////////////////////////////////
void MeshMatching::computeDescriptors(Mesh &mesh, vector<bool> &is_chosen_elems, vector<VertexDescriptor> &vec_vd, bool fill_vector) {
	int v_id=0;
	
	cout << "* Descriptors" << endl;
	if (mesh.p.size_of_vertices() !=is_chosen_elems.size()) {
		cout << "invalid is_chosen_elems size (" << is_chosen_elems.size() << ") in computeDescriptors" << endl;
		return;
	}

	int local_ring_size= getNoDescriptorRings(mesh,alg_featNoRings,alg_treatNoRingsAsSurfacePercentage);
	if (alg_treatNoRingsAsSurfacePercentage) {
		cout << " - estimating ring size for " << alg_featNoRings << "% = " << local_ring_size<< endl;	
	}	

	// populate the vertex description vectors
	FeatureGradientHistogram feature(local_ring_size, mesh.edge_avg, alg_featNoBinsCentroid, alg_featNoBinsGroups, alg_featNoBinsOrientations, alg_featSpatialInfluence);;

	if (alg_featType>3) {
		cout << "Disabled support for descriptor computation for this feature type for now" << endl;
		return;
		mesh.setMeshQuality(Mesh::Qual_Color_Deriv);
		feature=FeatureGradientHistogram(local_ring_size, mesh.edge_avg, alg_featNoBinsCentroid, alg_featNoBinsGroups, alg_featNoBinsOrientations, alg_featSpatialInfluence);
	}

	
	vector<float> descriptor;
	vector<float> vQualities;	
	VertexDescriptor vd;

	
	cout << " - recomputing gradients" << endl;	
	
	if (alg_featUsesDetScale) 
	{
		if (alg_featType==EQualColor) mesh.setMeshQuality(Mesh::Qual_Color);
		if (alg_featType==EQualMeanCurv) mesh.setMeshQuality(Mesh::Qual_Mean_Curv);	
		if (alg_featType==EQualGaussianCurv) mesh.setMeshQuality(Mesh::Qual_Gaussian_Curv);		
		if (alg_featType==EQualImported) mesh.setMeshQuality(Mesh::Qual_Imported);			
	}
	else
	{
		mesh.setMeshQuality(Mesh::Qual_Quality_Deriv);				
	}

	// initializations 
	for(int i=0;i<mesh.p.size_of_vertices();i++) 
	{
		vQualities.push_back(0.0f);	 // create the vQualities vector
		if (fill_vector) 
		{
			vd.reset();
			vec_vd.push_back(vd);		// create the vec_vd vector
		}
		
	}
	
	int comp_desc=0;
	cout << "\r - computing descriptors for selected interest points";
	for(int i=0;i<alg_noConvolves;i++) {
		// save the quality
		v_id=0;
		for (Vertex_iterator vi = mesh.p.vertices_begin(); vi!=mesh.p.vertices_end(); vi++, v_id++)  vQualities[v_id] = vi->quality();

		// compute the gradient of the quality measure (needed for descriptor)
		if (alg_featUsesDetScale) mesh.setMeshQuality(Mesh::Qual_Quality_Deriv);	

		//compute the descriptors for this scale
		v_id=0;
		for (Vertex_iterator vi = mesh.p.vertices_begin(); vi!=mesh.p.vertices_end(); vi++, v_id++) 
        {
			if (alg_featUsesDetScale && (vi->values.size()==2*(alg_noConvolves+1)) && (vi->values[2*alg_noConvolves+1]!=i)) continue;
			if (is_chosen_elems[v_id]) 
            {
				cout << "\r - computing descriptors for selected interest points - " << comp_desc+1; 
				if (alg_featUsesDetScale) cout << " - scale " << i;
				cout.flush();
				feature.computeFeatureVector(descriptor,*vi);
				vd.setVertex(*vi);
				vd.setDescriptor(descriptor);
				if (isnan(vd.norm(2)))
				{
					cout << "\n Error (NAN) while computing descriptor " << v_id << " for scale " << i << endl;
				}
				if (fill_vector) vec_vd[v_id] = vd;			
				else vec_vd.push_back(vd);
				comp_desc++;
			}
		}		
		if (!alg_featUsesDetScale) break;
		// restore the quality
		v_id=0;
		for (Vertex_iterator vi = mesh.p.vertices_begin(); vi!=mesh.p.vertices_end(); vi++, v_id++)  vi->quality() = vQualities[v_id];

		//blur the quality
		for (int ii=0;ii<i/alg_convsPerScale+1;ii++) mesh.convolve(alg_convSigma,false,false);
		//mesh.convolve(pow(2.0,2*(i/alg_convsPerScale)/2.0)*alg_convSigma,false,false);
	}
	cout << endl;

	
	if (alg_featType>3) { // gotto make another run to compute descriptors for a new feature
		cout << "Disabled support for descriptor computation for this feature type for now" << endl;
		return;
		/*
		if (alg_featType==4) {
			mesh.setMeshQuality(Mesh::Qual_Mean_Curv_Deriv);
			feature=FeatureGradientHistogram(1, local_ring_size, alg_featNoBinsCentroid, alg_featNoBinsGroups, alg_featNoBinsOrientations, alg_featSpatialInfluence);
		}
		for(int i=0;i<vec_vd.size();i++) {
			if (vec_vd[i].isSet()) {
				feature.computeFeatureVector(descriptor,*vec_vd[i].vertex);
				vec_vd[i].appendDescriptor(descriptor);				
			}
		}
		 */
	}
}

		
//////////////////////////////////////////////////////////////////////////////////////////////////////
void MeshMatching::computeColourFromDescriptors(Mesh &mesh, vector<VertexDescriptor> &vec_vd)
{
	int v_id=0;

	//colour stats
	float minColor[3];
	float maxColor[3];	
	for(int i=0;i<3;i++)
	{	
		minColor[i] = 1.0f;
		maxColor[i] = 0.0f;
	}

	// compute colours	
	for (Vertex_iterator vi = mesh.p.vertices_begin(); vi!=mesh.p.vertices_end(); vi++, v_id++)  
		{
			vector<float> desc = vec_vd[v_id].descriptor;
			int NN=desc.size()/3;			
			float total = 0.0f;
			for(int i=0;i<desc.size();i++) total+=desc[i];
				
			// compute the mean per colour channel
			for(int c=0;c<3;c++)
			{
				float colTotal=0.0f;
				for(int i=0;i<NN;i++)
					colTotal+=desc[c*NN+i];
				vi->color[c] = colTotal/(total+0.0000001f);
				minColor[c]=min(minColor[c],vi->color[c]);
				maxColor[c]=max(maxColor[c],vi->color[c]);				
			}
		}
	
	//normalize colours
	for (Vertex_iterator vi = mesh.p.vertices_begin(); vi!=mesh.p.vertices_end(); vi++, v_id++)  
	{
		for(int c=0;c<3;c++)
		{
			vi->color[c] = (vi->color[c] - minColor[c])/(maxColor[c] - minColor[c]+ 0.0000001f);
		}
	}	
}
										  
//////////////////////////////////////////////////////////////////////////////////////////////////////
void MeshMatching::computeFeaturesDescriptors(Mesh &mesh, vector<bool> &vDet, vector<VertexDescriptor> &vDesc, bool bComputeAllDescriptors) {
	CGAL::Timer time_elapsed;
	time_elapsed.start();
	
	computeFeatures(mesh, vDet);
	if (bComputeAllDescriptors)
	{
		vector<bool> vTmpDet;
		for(int i=0;i<vDet.size();i++) vTmpDet.push_back(true);
		cout << " - computing all descriptors" << endl;
		computeDescriptors(mesh, vTmpDet, vDesc, alg_opMode=='E');
	}
	else
		computeDescriptors(mesh, vDet, vDesc, alg_opMode=='E');
	
	cout << " - elapsed  "  << ( ( int ) time_elapsed.time() ) /60 << ":" << ( ( int ) time_elapsed.time() ) %60 << endl;		
	
}


	
//////////////////////////////////////////////////////////////////////////////////////////////////////
int findMatchWithinGeoDist(int source_id, Mesh&mesh, vector<bool> &vDet, float geo_ball) {
	Vertex *el = mesh.getVertexById(source_id);
	std::vector< std::pair<Vertex*,float> > elems = mesh.getGeodesicNeighbourhood(*el,geo_ball,true);

	for(int i=0;i<elems.size();i++) {
		int id=elems[i].first->id;
		if (vDet[id]==true)
			return id;
	}
	return -1;
	
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
void MeshMatching::generateGroundtruthMatches(Mesh & m1, Mesh &m2, std::vector<int> &vMatch,  GTGenerator eGenMode, float fModeParam) {
	if (eGenMode == EGTAutoDetect)
	{
		if (m1.p.size_of_vertices()==m2.p.size_of_vertices())
		{	
			eGenMode = EGTOneToOne;
			fModeParam = 0;			
		}
		else
		{
			eGenMode = EGTGeodesic;
			fModeParam = alg_geodesicBallGT;			
		}
	}
	
	
	
	if (eGenMode==EGTOneToOne) // 1-to-1 vertex correspondence
	{
		assert(m1.p.size_of_vertices()==m2.p.size_of_vertices());
		for(int i=0;i<m1.p.size_of_vertices();i++)
			vMatch.push_back(i);
	}
	else
	if (eGenMode==EGTGeodesic)
	{
		for ( Vertex_iterator it = m1.p.vertices_begin(); it != m1.p.vertices_end(); ++it )
		{
			Vertex *el = &(*m2.closestVertex(it->point()));		
			float dist = v_norm(it->point() - el->point());
			if (dist < fModeParam)
				vMatch.push_back(el->id);
			else
				vMatch.push_back(-1);
		}
	}
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
void MeshMatching::evaluateDetectorDescriptorWithGT(vector<bool> &vDet1, vector<VertexDescriptor> &vDesc1, 
                                                    vector<bool> &vDet2, vector<VertexDescriptor> &vDesc2, 
                                                    vector<int> &vMatches, bool bFullDescriptorMatch, Mesh &mesh, float geo_ball, 
                                                    float &det_rep, float &desc_norm) {
	// it assumes that both vectors are full and it computes the repeatability of the descriptor
	int detections_found=0;
	int total_detections=0;	
	int total_descriptions=0;		
	float dist, dist2;
	desc_norm = 0.0f;
	float desc_norm2=0.0f;
	if (  (vDet1.size()!=mesh.p.size_of_vertices()) || (vDet1.size()!=vDesc1.size()) || (vDet2.size()!=vDesc2.size()) ) 
    {
		cout << "ERROR in evaluateDetectorDescriptorWithGT:" << endl;
		cout << "-->> Size of detectors (" << vDet1.size() <<"," << vDet2.size() << ")" << endl;
		cout << "-->> Size of descriptors (" << vDesc1.size() <<"," << vDesc2.size() << ")" << endl;
		cout << "-->> No_of_vertices=" <<mesh.p.size_of_vertices() <<  endl;		
		return;
	}

    // distance bins
    int iNoBins = 20;
    int iBinSum = 0;
    float fBinWidth = 1.0f/iNoBins;
    std::vector<int> vDistBins;
    for(int i=0;i<iNoBins + 1;i++)
        vDistBins.push_back(0);
    
    
	for(int vd2_id=0;vd2_id<vDet2.size();vd2_id++) 
	{
		int vd1_id = -1;
		if (vDet2[vd2_id]) 
		{
			//detector
			if (vMatches[vd2_id]!=-1) vd1_id = findMatchWithinGeoDist(vMatches[vd2_id],mesh,vDet1,geo_ball);
			if (vd1_id!=-1)	detections_found++;
			
			total_detections++;
		}
		
		// descriptor
		
		if (bFullDescriptorMatch)  vd1_id = vMatches[vd2_id];
		if ((vd1_id!=-1) && (vDesc1[vd1_id].isSet()) && (vDesc2[vd2_id].isSet()) )
		{            
			float angle = vDesc1[vd1_id].distanceTo(vDesc2[vd2_id],0)/vDesc1[vd1_id].norm(2)/vDesc2[vd2_id].norm(2);
			angle = std::max(-1.0f,std::min(1.0f,angle));
			dist= acos(angle) /PI;
			dist2= vDesc1[vd1_id].distanceTo(vDesc2[vd2_id],1)/ (vDesc1[vd1_id].norm(2) + vDesc2[vd2_id].norm(2));					
			desc_norm+= dist;
			desc_norm2+=dist2;
            
            //bin the result
            int iBinId=(int)ceil(dist/fBinWidth)-1;
            vDistBins[iBinId]++;
            iBinSum++;
            
			total_descriptions++;					
			//printf("matched d(%d,%d)=%.3f %.3f\n",vd1_id,vd2_id,dist,dist2);
			if (isnan(dist))
			{	
				cout << "desc1=";	vDesc1[vd1_id].print(); cout << endl;
				cout << "desc2=";	vDesc2[vd2_id].print(); cout << endl;
				cout << "norm1=" << vDesc1[vd1_id].norm(2) << endl;
				cout << "norm2=" << vDesc2[vd2_id].norm(2) << endl;
				cout << "dist=" << dist << endl;
				cout << "dist2=" << dist2 << endl;

			}
            if (0)
            {
                cout << "==============================================" << endl;
                cout << "d1: "; vDesc1[vd1_id].print(); cout << endl;
                cout << "norm=" << vDesc1[vd1_id].norm(2) << endl;
                cout << "d2: "; vDesc2[vd2_id].print(); cout << endl;
                cout << "norm=" << vDesc2[vd2_id].norm(2) << endl;
                cout << "angle=" << angle << " dist=" << dist << " dist2=" << dist2 << " bin=" << iBinId << endl;
            }
		}
		
	}

	det_rep = detections_found*1.0f/(total_detections+0.001f);
	desc_norm = desc_norm*1.0f/PI/(total_descriptions+0.001f);
	desc_norm2 = desc_norm2*1.0f/(total_descriptions+0.001f);	

	cout << " - # correct detections = " << detections_found <<", out a total of " << total_detections << endl;	
	cout << " - # descriptors   =  " << total_descriptions << endl;	
	cout << " - L2 norm = " << desc_norm2 << endl;
	cout << " - dot product norm = " << desc_norm << endl;	
    cout << " - Precision / Recall for the Descriptor distance threshold: " << endl;	
    cout << "   1-Prec : ";
    for(int i=0;i<iNoBins ;i++)
        printf("%.2f ",(1+i)*fBinWidth);
    cout << endl;
    cout << "   Recall : ";
    
    if (iBinSum > 0)
    {
        char filename[1000];
        sprintf(filename,"%s.recall",alg_saveOutputFile);
        ofstream out(filename);
        if (!out) {
            std::cerr << "Error : Unable to write '" << filename << "'" << std::endl;
            return;
        }
 
        float fSum=0.0f;
        for(int i=0;i<iNoBins ;i++)
        {
            fSum+=vDistBins[i];
            float fRatio = fSum * 1.0f / iBinSum;
            printf("%.2f ",fRatio);
            out << fRatio << " ";
        }
        out << ";" << endl;
    }
    cout << endl;

	
}



//////////////////////////////////////////////////////////////////////////////////////////////////////
// execute the algorithm
void MeshMatching::run() {

	vector<bool> vDet1, vDet2;
	vector<VertexDescriptor> vDesc1, vDesc2;


	//	float bbox_max_movement_percentage=0.3;

	if ((alg_featType==EQualMeanCurv) || (alg_featType==EQualGaussianCurv)) {
        int curvature = alg_curvNoRings;
		mesh1.curv_comp_method = Mesh::Curv_Dong;
		mesh1.curv_comp_neigh_rings = curvature; 
		mesh2.curv_comp_method = Mesh::Curv_Dong;
		mesh2.curv_comp_neigh_rings = curvature;
	}
	
//	if (alg_opMode=='E') alg_featComputeAll = true;
	
	mesh1.loadFormat(alg_srcMeshFile,true);
	alg_geodesicBallGT = max((float)(2.0f*mesh1.edge_avg),mesh1.getSurfacePercentageRadius(2)); //getNoDescriptorRings(mesh1,0.5,true) * mesh1.edge_avg;
	if (alg_opMode=='E')
		printf("* GT geodesic radius = %.2f avg_edge\n",alg_geodesicBallGT/mesh1.edge_avg);
	
	if (alg_noise) {
		vector<int> vMatches;
		Mesh m = mesh1;
		Mesh meshTopology;
		if (alg_noiseSigmaGeometryTopology>0) meshTopology.loadFormat("./datasets/verify/plane-two-sided-slim.off",false);
		
		MeshNoiser::noiseColour(mesh1,alg_noiseSigmaColour,MeshNoiser::NoiseUniform);
		MeshNoiser::noiseColour(mesh1,alg_noiseSigmaColourShot,MeshNoiser::NoiseSaltAndPepper);
		MeshNoiser::noiseGeom(mesh1,alg_noiseSigmaGeometry,MeshNoiser::NoiseUniform);
		MeshNoiser::noiseGeom(mesh1,alg_noiseSigmaGeometryShot,MeshNoiser::NoiseSaltAndPepper);		
		MeshNoiser::noiseGeomRotate(mesh1,alg_noiseSigmaGeometryRotate,MeshNoiser::NoiseUniform);		
		MeshNoiser::noiseGeomScale(mesh1,alg_noiseSigmaGeometryScale);		
		MeshNoiser::noiseGeomLocalScale(mesh1,alg_noiseSigmaGeometryLocalScale);
		MeshNoiser::noiseGeomSampling(mesh1,alg_noiseSigmaGeometrySampling);
		MeshNoiser::noiseGeomHoles(mesh1,alg_noiseSigmaGeometryHoles,getNoDescriptorRings(mesh1,5,true));		
		MeshNoiser::noiseGeomHoles(mesh1,alg_noiseSigmaGeometryMicroHoles,3);		

		MeshNoiser::noiseGeomTopology(mesh1,alg_noiseSigmaGeometryTopology,meshTopology);
		
/*		mesh1.noise(alg_noiseSigmaGeometry,Mesh::ModifGeomNoise);
		mesh1.noise(alg_noiseSigmaGeometryShot,Mesh::ModifGeomShotNoise,2);
		mesh1.noise(alg_noiseSigmaGeometryRotate,Mesh::ModifGeomRotate);
		mesh1.noise(alg_noiseSigmaGeometryScale,Mesh::ModifGeomScale);
		mesh1.noise(alg_noiseSigmaGeometryLocalScale,Mesh::ModifGeomLocalScale);
		mesh1.noise(alg_noiseSigmaGeometrySampling,Mesh::ModifGeomSampling);
		mesh1.noise(alg_noiseSigmaGeometryHoles,Mesh::ModifGeomHoles,);
		mesh1.noise(alg_noiseSigmaGeometryMicroHoles,Mesh::ModifGeomMicroHoles,3);		
*/
		cout << " - noising data" << endl;
		cout << "        colour_noise=" << alg_noiseSigmaColour << endl;
		cout << "        alg_noiseSigmaColour=" << alg_noiseSigmaColour << endl;		
		cout << "        alg_noiseSigmaColourShot=" << alg_noiseSigmaColourShot << endl;		
		cout << "        alg_noiseSigmaGeometry=" << alg_noiseSigmaGeometry << endl;		
		cout << "        alg_noiseSigmaGeometryShot=" << alg_noiseSigmaGeometryShot << endl;		
		cout << "        alg_noiseSigmaGeometryRotate=" << alg_noiseSigmaGeometryRotate << endl;		
		cout << "        alg_noiseSigmaGeometryScale=" <<  alg_noiseSigmaGeometryScale << endl;		
		cout << "        alg_noiseSigmaGeometryLocalScale=" << alg_noiseSigmaGeometryLocalScale << endl;		
		cout << "        alg_noiseSigmaGeometrySampling=" << alg_noiseSigmaGeometrySampling << endl;		
		cout << "        alg_noiseSigmaGeometryHoles=" << alg_noiseSigmaGeometryHoles << endl;		
		cout << "        alg_noiseSigmaGeometryMicroHoles=" << alg_noiseSigmaGeometryMicroHoles << endl;		
		cout << "        alg_noiseSigmaGeometryTopology=" << alg_noiseSigmaGeometryTopology << endl;		
		cout << " - saving  noised mesh as mesh1_noised.coff"  <<endl;	
		cout << " - saving matches between original mesh and noised mesh as mesh1_matches.txt"  <<endl;			
		mesh1.saveFormat("mesh1_noised.coff");
		
		generateGroundtruthMatches(mesh1, m,vMatches,EGTAutoDetect, 0);
		saveGroundtruthMatches("mesh1_matches.txt",vMatches);
		
		if (alg_opMode=='N') return;
	}

	
	
/*	float *bbox = mesh1.getBoundingBox();
	float bbox_max_dim = max(abs(bbox[3]-bbox[0]),max(abs(bbox[4]-bbox[1]),abs(bbox[5]-bbox[2])));
	cout << " - estimating ring size with bounding box (10%)=" << round(bbox_max_dim*0.1/mesh1.edge_avg) << endl;
*/
		
	// compute colour for each descriptor
	if (alg_opMode=='C')
	{
		cout << " - compute descriptors for all vertices"<< endl;
		alg_noConvolves = 1;
		alg_featUsesDetScale = false;
		for(int i=0;i<mesh1.p.size_of_vertices();i++)
			vDet1.push_back(true);
		computeDescriptors(mesh1, vDet1, vDesc1, true);
		
		cout << " - descriptors: no(mesh1)=" << vDesc1.size() << endl;
		cout << " - computing mesh colours based on descriptors" <<endl;		
		computeColourFromDescriptors(mesh1, vDesc1);
		mesh1.saveFormat("mesh1_descriptor_colour.coff");
		return;
		
	}
	else
	{
		if (strcmp(alg_srcMeshDescFile,"0")==0)
			computeFeaturesDescriptors(mesh1,vDet1, vDesc1,alg_featComputeAll);
		else 
			loadFeatureDescriptorList(alg_srcMeshDescFile, mesh1, vDet1, vDesc1);
	}


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

	if (alg_opMode=='F') { //compute features on the 1st mesh and exit
		cout << " - descriptors: no(mesh1)=" << vDesc1.size() << endl;
		//saveOutput
		if (alg_saveOutput) {
			cout << " - saving to " << alg_saveOutputFile << " (all known formats .points, .feat, .det_desc, .desc)" <<endl;
			for(int i=0;i<5;i++)
				saveFeatureDescriptorList(vDet1,vDesc1,alg_saveOutputFile,SaveDescriptorFormat(i));
		}
		return;
	}

	mesh2.loadFormat(alg_dstMeshFile,true);

	if (strcmp(alg_dstMeshDescFile,"0")==0)
		computeFeaturesDescriptors(mesh2,vDet2,vDesc2,alg_featComputeAll);
	else
		loadFeatureDescriptorList(alg_dstMeshDescFile, mesh2, vDet2, vDesc2);
	
	if (alg_opMode=='M') 
		performMatching(vDet1, vDesc1, vDet2, vDesc2);

	if (alg_opMode=='E') 
		performEvaluation(vDet1, vDesc1, vDet2 ,vDesc2);

}


//////////////////////////////////////////////////////////////////////////////////////////////////////
void MeshMatching::performEvaluation(vector<bool> &vDet1, vector<VertexDescriptor> &vDesc1, vector<bool> &vDet2, vector<VertexDescriptor> &vDesc2) {
	float detector_ratio;
	float descriptor_error;
	
	cout << "* Evaluating results" << endl;
/*
	char *feat1_name= "mesh1.feat";
	cout << " - saving  descriptor file " << feat1_name  <<endl;
	ofstream of_feat1(feat1_name);
	if (!of_feat1) {
		std::cerr << "Error : Unable to write '" << feat1_name << "'" << std::endl;
		return;
	}
	saveFeatureDescriptorList(vDet1,vDesc1,of_feat1);

	char *feat2_name= "mesh2.feat";
	cout << " - saving  descriptor file " << feat2_name  <<endl;
	ofstream of_feat2(feat2_name);
	if (!of_feat2) {
		std::cerr << "Error : Unable to write '" << feat2_name << "'" << std::endl;
		return;
	}
	saveFeatureDescriptorList(vDet2, vDesc2,of_feat2);
*/	
	
	vector<int> vMatches;
	if (strlen(alg_matchesDstSrcFile) > 1)
		loadGroundtruthMatches(alg_matchesDstSrcFile,vMatches);
	if (vMatches.size()==0)
		generateGroundtruthMatches(mesh2, mesh1, vMatches, EGTAutoDetect, 0);

	evaluateDetectorDescriptorWithGT(vDet1,vDesc1,vDet2, vDesc2, vMatches, alg_featComputeAll, mesh1, alg_geodesicBallGT,detector_ratio,descriptor_error);
	cout << " - detector repeatability " << detector_ratio << endl;
	cout << " - descriptor L-2 distance " << descriptor_error << endl;	
	
	cout << " - saving  evaluation results to  file " << alg_saveOutputFile  <<endl;
	ofstream of_eval(alg_saveOutputFile);
	if (!of_eval.good()) {
		std::cerr << "Error : Unable to write '" << of_eval << "'" << std::endl;
		return;
	}
	else
	{
		int noDetections=0;
		for(int i=0;i<vDet2.size();i++) 
			if (vDet2[i]) noDetections++;
		of_eval << detector_ratio << " " << descriptor_error << " " << noDetections << endl;	
	}
	
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
void MeshMatching::performMatching(vector<bool> &vDet1, vector<VertexDescriptor> &vec_vd1, vector<bool> &vDet2, vector<VertexDescriptor> &vec_vd2)
{
	bool evaluate_matches=false;

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

	
	int distType = 1; // 0 - dot product; 1 - euclidean norm
//	float max_movement=bbox_max_movement_percentage*bbox_max_dim; 
	if (vec_vd2.size()>0)
	for (int i=0;i<vec_vd1.size();i++) {
		float min_dist, min_2nd_dist;
		int min_pos;
		
		//find closest match
		min_dist=MAXFLOAT;
		for (int j=0;j<vec_vd2.size();j++) {
			float dist=vec_vd1[i].distanceTo(vec_vd2[j],distType);
			if (min_dist>dist) { min_dist=dist; min_pos=j;}
		}

		//find closest match back
		float min_dist_back=MAXFLOAT;
		int min_pos_back=1;
		for (int ii=0;ii<vec_vd1.size();ii++) {
			float dist=vec_vd1[ii].distanceTo(vec_vd2[min_pos],distType);
			if (min_dist_back>dist) { min_dist_back=dist; min_pos_back=ii;}
		}

			
		//find 2nd closest match
		min_2nd_dist=MAXFLOAT;
		for (int j=0;j<vec_vd2.size();j++) {
			float dist=vec_vd1[i].distanceTo(vec_vd2[j],distType);
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
	if (alg_saveOutput) 
    {
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
			ofstream fs(alg_saveOutputFile);
				if (!fs) {
					std::cerr << "Error : Unable to write '" << alg_saveOutputFile << "'" << std::endl;
					return;
				}


			saveFeatureDescriptorList(vDet1,vec_vd1,fs);
			saveFeatureDescriptorList(vDet2,vec_vd2,fs);
			//matches
			fs << matches.size() << endl;
			for(int i=0;i<matches.size();i++)
				fs << matches_index[i].first << " " << matches_index[i].second << endl;
		}

		if (evaluate_matches) 
        { // save error histogram
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

