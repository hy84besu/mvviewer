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
	FeatureGradientHistogram(int no_rings_, float avg_edge_size_, int no_bins_centroid_, int no_bins_groups_ , int no_bins_orientations_ , double spatial_influence_);

	Point rotatePointAroundVector(Vector axis, Point point, float angle);
	float angleWithRefVector(Vector x, Vector origin, Vector normal);
	Vector getGradient(Vertex& v);
	void computeFeatureVector(vector<float>& result, Vertex &v);

private:
	int no_rings;
	float avg_edge_size;
	int no_bins_centroid;
	int no_bins_groups;
	int no_bins_orientations;
	float spatial_influence;
};

#endif