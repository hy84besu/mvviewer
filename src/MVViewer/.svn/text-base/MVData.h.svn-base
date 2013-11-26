#ifndef _MVDATA_H_
#define _MVDATA_H_

#include "Camera.h"
#include "Mesh.h"
#include "CImg_mod.h"
#include "Resample.h"
#include "RGB.h"

#include <assert.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>

using namespace std;
using namespace cimg_library;

template<typename T> inline T minmod(const T &a,const T &b) { return a*b<=0?0:(a>0?(a<b?a:b):(a<b?b:a)); }
template<typename T> inline void assign_if_greater(T &x,const T &y){ if(x<y)x=y; }
template<typename T> inline void assign_if_lower(T &x,const T &y){ if(x>y)x=y; }


// a class for reading the multiview stereo evaluation data
class MVData {

public:	
	// calibration data
	bool data_loaded;
	int ncameras;
	Camera *cameras;
	//image info
	int D; //image dimension
	//int BG_R,BG_G,BG_B;
	rgb bg_color;
	float bg_variance;
	bool is_rgb;
	bool crop_using_masks;
	CImg<> *images;
	CImg<unsigned char> *silhouettes; //white inside, black outside

	// mesh data
	Mesh mesh;
	
	Polyhedron visual_hull;	
	

	MVData(float bg_r=0.0, float bg_g=0, float bg_b=0, float bg_var=30);
	~MVData();
	void downscaleInputs(int scale);
	void blurInputs(float sigma=3.0f);
	bool loadCameras(int ncamera, const char* filePattern);
	bool loadImages(int ncamera, const char* filePattern, const char *mask_filePattern=NULL, bool make_grayscale=false, bool crop_using_masks=false);
	
	void generateCameraList(int n, int counterStart, int counterIncrement, const char* cameraListFile);
	bool loadData(int _n, int counterStart, int counterIncrement, const char* cameraListFile, const char *_cameras, const char *_pmesh, const char *_images, const char *_mask_images, bool make_grayscale=false, const char* motionVecFile="0", const bool crop_using_masks=false);
	void clearData();
	void cropBackgroundFromImages();
	void downScaleInputs(int scale);

	//heuristics for Mesh startup level
	float computeDesiredEdgeSize(double scale, float targetEdgeInPixels);
	float computeDesiredLevel(double targetEdge, float targetEdgeInPixels);

	int imageWidth(int i);
	int imageHeight(int i);
	int imageMaxWidth();
	int imageMaxHeight();	
	
private:
	bool camera_list_generated;
	vector<int> camera_list;

};

#endif
