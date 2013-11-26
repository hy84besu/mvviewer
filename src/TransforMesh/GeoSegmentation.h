//
// C++ Interface: GeoSegmentation
//
// Description: 
//
//
// Author: Kiran Varanasi <varanasi@monoceros>, (C) 2008
//
// Copyright: See COPYING file that comes with this distribution
//
//

#ifndef GEOSEGMENT_H
#define GEOSEGMENT_H

#include <QThread>

#include "Mesh.h"
#include <algorithm>
#include "GeoIntegral.h"

class GeoSegment {

  public:

	GeoSegment() { 
		active = true;
		color[0] = color[1] = color[2] = 0; 
		size = 0; id = -1; 
		root = NULL; notches.clear();
		neigh.clear(); neighdist.clear();
	}
	GeoSegment(Vertex_handle vi, int _id) {
		active = true;
		color[0] = color[1] = color[2] = 0; 
		size = 1; id = _id; 
		root = vi; notches.clear();
		neigh.clear(); neighdist.clear();
	}

	void addNeighbour(int i, float dist) {
		int p; for(p=0; p < neigh.size(); p++) { 
			if(neigh[p] == i) break;
		}
		if (p == neigh.size()) {
			neigh.push_back(i);
			neighdist.push_back(dist);
		} else { 
			if(neighdist[p] > dist) 
				neighdist[p] = dist;
		}
	}

	void deleteNeighbour(int i) {
		vector<int>::iterator p = find(neigh.begin(), neigh.end(), i);
		if (p != neigh.end()) {
			neigh.erase(p);
			neighdist.erase(neighdist.begin() + std::distance(p, neigh.begin()));
		}
	}

	int size;
	int id;
	Vertex_handle root;
	vector<Vertex_handle> notches; // local extrema of geoIntegral inside the segment
				// local-segment-wise geoIntegral is stored in MeshVertex->weight
	vector<int> neigh;
	vector<float> neighdist;
	float color[3];
	bool active;

};

class GeoSegmentation: public QThread {

Q_OBJECT 

public:
	GeoSegmentation();
	GeoSegmentation(Mesh *m);
	void load(Mesh *m); 
	~GeoSegmentation();
	void cleanUp();
	void getGeoIntegral(); 
	void setColors();
	void setColors(int colorStyle);

	GeoIntegral ginteg;
	Mesh *m;
	vector<Vertex_handle> sortedVerts; 
	int numSegments;
	double epsilon; // the threshold for epsilon-convexity
	
	void makeSegments(double _epsilon); 

signals:
	void stepFinished();
	
private:
	Vertex_handle *vertex; 	 
	float *convexity;
	int *parent; 
	int *vertSegment;

	std::map<Kernel::Point_3, Kernel::Vector_3> point_mapping; 

	deque< pair<Vertex_handle,int> > segVerts; 
	void buildSegment(Vertex_handle vi, int pid); 
	int changeSegmentId(Vertex_handle v, int new_id);

	vector<GeoSegment *> segments;

	// In the MeshVertex variables the following information will be stored
	// float qual; // which stores the geodesic integral
protected:
	virtual void run();
};


#endif
