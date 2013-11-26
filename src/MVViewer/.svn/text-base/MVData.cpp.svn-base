/*
 *  MVData.cpp
 *  src
 *
 *  Created by Andrei Zaharescu on 28/11/06.
 *  Copyright 2006 __MyCompanyName__. All rights reserved.
 *
 */

#include "MVData.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////
MVData::MVData(float bg_r, float bg_g, float bg_b, float bg_var) {
	ncameras=0;
	D =1;
	bg_color[0]=bg_r;
	bg_color[1]=bg_g;
	bg_color[2]=bg_b;
	bg_variance = bg_var;
	data_loaded = false;
	camera_list_generated=false;
	images = NULL;
	silhouettes = NULL;
	cameras = NULL;
	
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
MVData::~MVData() {
	clearData();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
void MVData::blurInputs(float sigma) {
	cout << "Blurring input images with sigma=" << sigma << endl;
	if (images)	
		for (int i=0;i<ncameras;i++) {
			if (images[i]) images[i].blur(sigma);
		}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
void MVData::downScaleInputs(int scalefactor) {
	cout << "old image sizes are: (" << imageMaxWidth()  << "," << imageMaxHeight() << ")" << endl;
	float scale=1<<scalefactor;

	for (int i=0;i<ncameras;i++) {
		//scale the cameras
		//		cameras[i].ScaleWorld(float(1<<scale));
		cameras[i].ScaleImage(1 / scale);
		//		cameras[i].Normalize();
		if (images) {
			CImg<> tmpImg = CImg<>(round(images[i].dimx()/scale),round(images[i].dimy()/scale),1,D);				
			//scale the images
			Resample(images[i],tmpImg);
			images[i] = tmpImg;
/*			char tmpFilename[200];
			sprintf ( tmpFilename,"./intermediate_outputs/scaled_input-%04d.png",camera_list[i]);
			tmpImg.save ( tmpFilename );
*/
		}
		if (silhouettes) {
			CImg<> tmpImg = CImg<>(round(images[i].dimx()/scale),round(images[i].dimy()/scale),1,1);
			//scale the images
			Resample(silhouettes[i],tmpImg);
			silhouettes[i] = tmpImg;
		}
		
	}
	cout << "new image sizes are: (" << imageMaxWidth()  << "," << imageMaxHeight() << ")" << endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
bool MVData::loadCameras(int ncamera, const char* filePattern) {
	// Chargement des cameras et des images
	cameras = new Camera[ncamera];
    	cout << "* Loading cameras: "; cout.flush();
  	for (int i=0;i<ncamera;i++) {
		// Chargement des cameras 
		char s[255];
		sprintf(s,filePattern,camera_list[i]);
		cout << "[" << s << " "; cout.flush();
		if (cameras[i].Load(s)) {
			ncameras=0;
			return false;
		}
		cameras[i].id=camera_list[i];
		cout << i<< "] "; cout.flush();
	}
    	cout << "OK" << endl;
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
bool MVData::loadImages(int ncamera, const char* filePattern, const char *mask_filePattern, bool make_grayscale, bool crop_using_masks_) {
	
	if (filePattern==NULL) return true;
	// Chargement des images
	images = new CImg<> [ncamera];
	crop_using_masks = crop_using_masks_;
	cout << "* Loading images: ";
	cout.flush();
	for (int i=0;i<ncamera;i++) {
		// Chargement des cameras
		char s[255];
		sprintf(s,filePattern,camera_list[i]);
		cout << "[ " << s << " ";
		cout.flush();

		if (make_grayscale) {
			CImg<> tmpImg;
			tmpImg.load(s);
			images[i] = CImg<>(tmpImg.dimx(),tmpImg.dimy(),1,1);
			cimg_forXY(tmpImg,x,y) {
				images[i](x,y,0,0) = (tmpImg(x,y,0,0) + tmpImg(x,y,0,1) + tmpImg(x,y,0,2))/3;
			}		
		}
		else
			images[i].load(s);

		if (images[i].size() == 0) {
			ncameras=0;
			return false;			
		}
		if (cameras) {
			cameras[i].width=images[i].dimx();
			cameras[i].height=images[i].dimy();			
		}

		cout << "] ";
		cout.flush();
	}

	D = images[0].dim;
	is_rgb = (D ==3);
	cout << " of sizes:" << imageMaxWidth() << "x" << imageMaxHeight() << "x" << D << " OK" << endl;
	
//	return true;
	
	if (mask_filePattern && (strlen(mask_filePattern)>1)) {
		cout << "* Loading masks: ";
		silhouettes = new CImg<unsigned char> [ncamera];		
//		CImg<unsigned char> mask_image;
		for (int i=0;i<ncamera;i++) {
			// Chargement des cameras
			char s[255];
			sprintf(s,mask_filePattern,camera_list[i]);
			cout << "[ " << s << " ";
			cout.flush();
			silhouettes[i].load(s);
			if (silhouettes[i].size() == 0)
				return false;
			cout << "] ";
			cout.flush();
			if (silhouettes[i].dimv()==3) {
				CImg<unsigned char> tmpImg(silhouettes[i].dimx(),silhouettes[i].dimy(),1,1);
				cimg_forXY(silhouettes[i],x,y) {
					tmpImg(x,y,0,0) = (silhouettes[i](x,y,0,0) + silhouettes[i](x,y,0,1) + silhouettes[i](x,y,0,2))/3;
				}	
				silhouettes[i] = tmpImg;
			}	
		}

		if (crop_using_masks) cropBackgroundFromImages();
		cout << "OK" << endl;
	}


	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
void MVData::cropBackgroundFromImages() {
	for (int i=0;i<ncameras;i++) {
		//clear the background
		cimg_forXY(silhouettes[i],x,y) if (silhouettes[i](x,y,0,0) == 0) {
			images[i](x,y,0,0) = bg_color[0];
			if (is_rgb ) {
				images[i](x,y,0,1) = bg_color[1];
				images[i](x,y,0,2) = bg_color[2];
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
void MVData::generateCameraList(int n, int counterStart, int counterIncrement, const char* cameraListFile) {
	camera_list.clear();
	if (cameraListFile!=NULL) { // read the camera list from a file
		ifstream is (cameraListFile);
		int p;
		while (true) {
			if (is >> p) {
			camera_list.push_back(p);	
			}
			else 
				break;
		}
	}
	else {
		for (int i=0;i<n;i++)
			camera_list.push_back(i*counterIncrement+counterStart);
	
	}
	ncameras=camera_list.size();
	camera_list_generated=true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
bool MVData::loadData(int _n,  int counterStart, int counterIncrement, const char* cameraListFile, const char *_cameras, const char *_pmesh, const char *_images, const char *_mask_images, bool make_grayscale, const char *motionVecFile, const bool crop_using_masks ) {

	generateCameraList(_n,counterStart,counterIncrement,cameraListFile);
	
	bool result=loadCameras(ncameras,_cameras) && loadImages(ncameras,_images,_mask_images,make_grayscale) && mesh.loadFormat(_pmesh,true) && mesh.loadVectorField(motionVecFile,true) &&(data_loaded=true);
	return result;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
void MVData::clearData() {
	if (data_loaded) {
		if (images) delete [] images;	
		if (silhouettes) delete [] silhouettes;			
		if (cameras) delete [] cameras;
	}
	data_loaded=false;
}



//////////////////////////////////////////////////////////////////////////////////////////////////////
/* Given the target scale and the target edge size in pixels, the edge size in mm is returned */
float MVData::computeDesiredEdgeSize ( double the_scale, float targetEdgeInPixels )
{
	float desiredEdge = 0;
	float* bbox = mesh.getBoundingBox();
	Point object_center = Point ( ( bbox[0] + bbox[3] ) /2, ( bbox[1] + bbox[4] ) /2, ( bbox[2] + bbox[5] ) /2 );
	//	float box_variance = (abs(bbox[3] - bbox[0])/2 + abs(bbox[4] - bbox[1])/2) /2;
	for ( int i=0;i<ncameras;i++ )
	{
		Vector3 tmpV = cameras[i].Center();
		Point cam_center = Point ( tmpV ( 1 ),tmpV ( 2 ),tmpV ( 3 ) );
//		float ku = cameras[i].deltau / ( W/2 );
		float z = v_norm ( cam_center-object_center ); // TODO:make this more accurate
		//	    z = z - box_variance/2;
		float focal = cameras[i].alphau / pow ( 2, ( double ) the_scale );
		float targetEdge =  abs ( targetEdgeInPixels * z / focal );
		desiredEdge+=targetEdge;
	}
	return desiredEdge/ncameras;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
/* Given the target scale and the target edge size in pixels, the edge size in mm is returned */
float MVData::computeDesiredLevel ( double targetEdge, float targetEdgeInPixels )
{
	float desiredEdge = 0;
	int cameraCount=0;
	float* bbox = mesh.getBoundingBox();
	Point object_center = Point ( ( bbox[0] + bbox[3] ) /2, ( bbox[1] + bbox[4] ) /2, ( bbox[2] + bbox[5] ) /2 );
	float box_variance =  (abs(bbox[0] - bbox[3])/2 + abs(bbox[1] - bbox[4])/2 + abs(bbox[2] - bbox[5])/2)/3;
	for ( int i=0;i<ncameras;i++ )
	{
		Vector3 tmpV = cameras[i].Center();
		Point cam_center = Point ( tmpV ( 1 ),tmpV ( 2 ),tmpV ( 3 ) );
		float ku = cameras[i].deltau / ( cameras[i].width/2 );

		float z = v_norm ( cam_center-object_center ); // TODO:make this more accurate
		z = z - box_variance/3;
		float focal = cameras[i].alphau / ku ;
		float tmpTargetEdge =  abs ( targetEdge * focal / z ) ;
		desiredEdge+=log ( tmpTargetEdge/targetEdgeInPixels ) /log ( 2 );
		cameraCount++;
	}
	if (cameraCount!=0) desiredEdge/=cameraCount;
	return desiredEdge;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
int MVData::imageWidth(int i) {
	assert(i>=0 && i<ncameras);
	return images[i].dimx();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
int MVData::imageHeight(int i) {
	assert(i>=0 && i<ncameras);
	return images[i].dimy();	
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
int MVData::imageMaxWidth() {
	int the_max=0;
	for(int i=0;i<ncameras;i++) the_max=std::max(the_max,imageWidth(i));
	return the_max;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
int MVData::imageMaxHeight() {
	int the_max=0;
	for(int i=0;i<ncameras;i++) the_max=std::max(the_max,imageHeight(i));
	return the_max;
}
