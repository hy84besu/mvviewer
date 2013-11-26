#include "GeoSegmentation.h"
#include "ColorMap.h"

#define MIN_CONVEXITY 0.00006

bool geoIntegralSortPredicate(const Vertex_handle vi, const Vertex_handle vj)
{
	return vi->qual > vj->qual;
}


GeoSegmentation::GeoSegmentation()
{
	m = NULL; 
	sortedVerts.clear();
	convexity = NULL; 
	parent = NULL;  
	vertex = NULL;
	vertSegment = NULL; 
	segments.clear();
	numSegments = 0; 

	epsilon = 0.7; 
}

GeoSegmentation::GeoSegmentation(Mesh *_m) 
{
	m = _m; 
	int i=0, numVert=m->p.size_of_vertices(); 
	Vertex_handle vi;

	fprintf(stderr, "Initializing a mesh of %d vertices\n", numVert);

	getGeoIntegral();
	fprintf(stderr, "GeoIntegral computed \n");
	sortedVerts.clear();
	vertex = new Vertex_handle[numVert];
	convexity = new float[numVert];
 	parent = new int[numVert];
	vertSegment = new int[numVert]; 
	for(vi = m->p.vertices_begin(),i=0; vi != m->p.vertices_end(); vi++,i++) { 
		sortedVerts.push_back(vi);
		vertex[i] = vi; convexity[i]=MIN_CONVEXITY; parent[i]=-1; vertSegment[i]=-1;
	}
	std::sort( sortedVerts.begin(), sortedVerts.end(), geoIntegralSortPredicate );
	fprintf(stderr, "%d vertices sorted by GeoIntegral\n", i);

	segments.clear();
	numSegments = 0; 
}

GeoSegmentation::~GeoSegmentation()
{
	cleanUp();
}

void GeoSegmentation::load(Mesh *_m) 
{
	m = _m;
	int i=0, numVert=m->p.size_of_vertices(); 
 
	fprintf(stderr, "Initializing a mesh of %d vertices\n", numVert);
	getGeoIntegral();
	fprintf(stderr, "GeoIntegral computed \n");
	sortedVerts.clear();

	delete [] vertex; 
	vertex = new Vertex_handle[numVert];
	delete [] convexity; 	
	convexity = new float[numVert];
	delete [] parent; 
	parent = new int[numVert];
	delete [] vertSegment; 
	vertSegment = new int[numVert]; 

	point_mapping.clear();

	for(Vertex_handle vi = m->p.vertices_begin(); vi != m->p.vertices_end(); vi++) { 
		sortedVerts.push_back( vi );
		vertex[i] = vi; convexity[i]=MIN_CONVEXITY; parent[i]=-1; vertSegment[i]=-1; i++;
 
		point_mapping[vi->point()] = vi->normal(); 
	}
	std::sort( sortedVerts.begin(), sortedVerts.end(), geoIntegralSortPredicate );
	fprintf(stderr, "Vertices sorted by GeoIntegral\n");

	segments.clear();
	numSegments = 0; 
}


void GeoSegmentation::getGeoIntegral() 
{
	ginteg.load(m, 1);
	ginteg.patchWarshall( 4 );  
	fprintf(stderr, "geo-patches + ");
	ginteg.geoIntegral();
	fprintf(stderr, "geo-integral + ");

	ginteg.cleanUp();
	fprintf(stderr, "cleanup \n");
}

void GeoSegmentation::buildSegment(Vertex_handle vi, int pid)
{
	int i,id, numConflict=0; 
	vector<int> path; path.clear();
	Point interp, closept; 
	double intlen, intang, closelen, closeang; 
	float conxLen=0,compConvexity=0; 
	Vertex_handle vj; 

	segVerts.pop_front(); // dequeues "vi" from the front of the queue 

	if(vertSegment[pid] < 0 || vertSegment[pid] >= segments.size()) return; 
	GeoSegment *gseg = segments[vertSegment[pid]];
	if(gseg == NULL || gseg->root == NULL || vi == NULL) return; // should never arise

	//fprintf(stderr, "grow-from(%d) cur-size(%d) ", vi->id, segVerts.size());

	for(id=vi->id; id > 0 && id != gseg->root->id; id = parent[id]) {
		path.push_back(id); 
	}
	//fprintf(stderr, "path-size(%d) ", path.size());
	
	for(i=1; i < path.size(); i++) { 
		id = path[i]; 
		interp = vi->point() + (float)i/path.size() * (gseg->root->point() - vi->point());

		// check if "interp" is outside the shape 

		closept = m->closestPoint(interp); 
		closelen = v_norm(closept - interp);
		closeang = v_angle(interp - closept, point_mapping[closept]);

		if(closeang < PI/2 && closelen > epsilon) numConflict++; 		

		// check if "interp" satisfies the assumption over the path

		intlen = v_norm(interp - vertex[id]->point());
		intang = v_angle(interp - vertex[id]->point(), vertex[id]->normal());

		if(intang > PI/2 && intlen > conxLen) 
			conxLen = intlen; 
	}

	if(path.size() == 0 || !numConflict || numConflict < path.size()/4.0) 
		compConvexity = (epsilon-conxLen)/epsilon; 	

	ColorMap::hot(compConvexity, (float *)(vi->color));

	//fprintf(stderr, "numconflict(%d) conxlen(%1.3f) comp-conx(%1.3f) cur-conx(%1.3f) ", numConflict, conxLen, compConvexity, convexity[vi->id]);

	//if(compConvexity >= convexity[vi->id]) { 
	if(convexity[vi->id] <= MIN_CONVEXITY && compConvexity > MIN_CONVEXITY) {
		// add this vertex to the segment and check its neighbors
		convexity[vi->id] = compConvexity; 
		//vertSegment[vi->id] = gseg->id; 
		//parent[vi->id] = pid; 
		
		HV_circulator hv = vi->vertex_begin();
		do { 
			vj = hv->opposite()->vertex();
			if(vertSegment[vj->id] != gseg->id) {
				vertSegment[vj->id] = gseg->id; parent[vj->id] = vi->id;  
				segVerts.push_back( pair<Vertex_handle, int>(vj,vi->id) );  
		//		fprintf(stderr, "add(%d) ", vj->id);
			}
				// buildSegment(vj, vi->id);  // for a depth-first-search
			hv++;
		} while(hv != vi->vertex_begin()); 
	} else { 
		parent[vi->id] = -1; 
		vertSegment[vi->id] = -1; 
	}
	//fprintf(stderr, "\n");
	//emit stepFinished();
}

void GeoSegmentation::makeSegments(double _epsilon=0.7)
{
	int cur=0, numConsider=0, stepSize=30;
	Vertex_handle vroot; 
	GeoSegment *gseg = NULL; 

	epsilon = _epsilon; 

	setColors(); // make all vertices look blue to begin with
	fprintf(stderr, "Making Segments \n");
	while(cur < sortedVerts.size()) { 
		vroot = sortedVerts[cur];
		//fprintf(stderr, "svert(%d) ", cur);
		if(convexity[vroot->id] > MIN_CONVEXITY) { cur++; continue; } // already visited by a segment
		parent[vroot->id] = vroot->id; 
		vertSegment[vroot->id] = numSegments; 
		vroot->color[0]=1; vroot->color[1]=1; vroot->color[2]=1; 
		//vroot->weight = 0; 
		fprintf(stderr, "Segment(%d) with root at vertex(%d) \n", numSegments, vroot->id);

		gseg = new GeoSegment(vroot, numSegments++); 
		segments.push_back(gseg); 
		segVerts.clear(); 
		segVerts.push_back( pair<Vertex_handle, int>(vroot,vroot->id) ); 
		while(! segVerts.empty()) {
			buildSegment(segVerts.front().first, segVerts.front().second);
			numConsider++;
			if(numConsider % stepSize == 0) emit stepFinished();
		}
		setColors(); // update color after each segment finished
		emit stepFinished();
	}

	//setColors();
}

void GeoSegmentation::run()
{
	makeSegments(0.005);
}

void GeoSegmentation::setColors()
{
	Vertex_handle vi; 
	int i=0;
	int ring=6;
	float val;

	for(vi = m->p.vertices_begin(),i=0; vi != m->p.vertices_end(); vi++, i++) { 
		val = (float)(vertSegment[i]%ring+1.0)/ring;
		//val = (float)(vertSegment[i]+1.0)/segments.size();
		ColorMap::jet(val, (float *)vi->color );
	}
}

void GeoSegmentation::cleanUp()
{
	if(m == NULL) return;

	delete [] vertex;
	delete [] convexity;
	delete [] parent;

	point_mapping.clear();
	segVerts.clear();
	for(int i=0; i < segments.size(); i++) {
		delete segments[i];
	}
	segments.clear();
	
	delete [] vertSegment; 
}

