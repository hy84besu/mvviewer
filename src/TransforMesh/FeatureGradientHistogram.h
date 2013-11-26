//
// C++ Interface: FeatureGradientHist
//
// Description: 
//
//
// Author: Andrei Zaharescu <andrei.zaharescu@inrialpes.fr>, (C) 2008
//
// Copyright: See COPYING file that comes with this distribution
//
//

#ifndef FEATURE_GRADIENT_HISTOGRAM_H
#define FEATURE_GRADIENT_HISTOGRAM_H

#include "Mesh.h"
	
class FeatureGradientHistogram {
public:
	FeatureGradientHistogram(int feature_type_, int no_rings_, int no_bins_centroid_, int no_bins_groups_ , int no_bins_orientations_ , double spatial_influence_);

	Point rotatePointAroundVector(Vector axis, Point point, float angle);
	float angleWithRefVector(Vector x, Vector origin, Vector normal);
	Vector getGradient(Vertex& v, int feat_type);
	void computeFeatureVector(vector<float>& result, Vertex &v);
	void computeFeatureVector(vector<float>& result, Vertex &v, int feat_type);

private:
	int feature_type; // 0 - color; 1 - mean curvature; 2 - gaussian curvature; 3 - color + mean_curv
	int no_rings;
	int no_bins_centroid;
	int no_bins_groups;
	int no_bins_orientations;
	float spatial_influence;
};

#endif