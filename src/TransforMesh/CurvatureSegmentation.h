//
// C++ Interface: CurvatureSegmention
//
// Description: 
//
//
// Author: Andrei Zaharescu <zaharesc@octans>, (C) 2008
//
// Copyright: See COPYING file that comes with this distribution
//
//

#ifndef CURVATURE_SEGMENTATION_H
#define CURVATURE_SEGMENTATION_H
#include "Mesh.h"
#include <algorithm>


class CurvatureSegment {

  public:

	CurvatureSegment() {
		active = true;
	}

	void addNeighbour(int i) {
		vector<int>::iterator p = find(neigh.begin(), neigh.end(), i);
		if (p == neigh.end())
			neigh.push_back(i);
	}

	void deleteNeighbour(int i) {
		vector<int>::iterator p = find(neigh.begin(), neigh.end(), i);
		if (p != neigh.end())
			neigh.erase(p);
	}

	int size;
	int id;
	Facet_handle facet;
	vector<int> neigh;
	float color[3];
	bool active;
};

class CurvatureSegmention {
public:
	CurvatureSegmention(Mesh *m, bool colorFacets);
	~CurvatureSegmention();
	void cleanUp();
	void segment(float curv_threshold, int island_threshold);

private:
	int changeSegmentId(Facet_handle f, int new_id);

	Mesh *m;
	vector<CurvatureSegment*> segments;
	bool colorFacets;
};


#endif