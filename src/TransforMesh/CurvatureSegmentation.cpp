//
// C++ Implementation: CurvatureSegmention
//
// Description: 
//
//
// Author: Andrei Zaharescu <zaharesc@octans>, (C) 2008
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "CurvatureSegmentation.h"



CurvatureSegmention::CurvatureSegmention(Mesh* _m, bool _colorFacets) {
 	m = _m;
	colorFacets = _colorFacets;
}

void CurvatureSegmention::cleanUp() {
	for(int i=0;i<segments.size();i++)
		delete segments[i];
}

CurvatureSegmention::~CurvatureSegmention() {
	cleanUp();
}


/////////////////////////////////////////////////////////////////////////////////////////////
void getVertexStats(Halfedge_handle h, float* accumulators, float bound1, float bound2) {

	accumulators[0] = std::max(0.0f,std::min((h->vertex()->mean_curvature() - bound1)/(bound2-bound1),1.0f));
//	angle = 1;
//	angle = v_normalized(h->vertex()->curvature())*h->vertex()->normal();
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool CurvatureSegmentSortPredicate(const CurvatureSegment* lhs, const CurvatureSegment* rhs)
{
  return lhs->size < rhs->size;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void computeVectorStats( vector<float>& v, float& mean, float& std_dev, float& std_dev_max) {
	mean = 0;
	std_dev = 0;
	std_dev_max =0;
	for(int k=0;k<v.size();k++)
		mean += v[k];
	mean /= v.size();
	for(int k=0;k<v.size();k++) {
		float tmp = SQUARED(mean-v[k]);
		std_dev += tmp;
		std_dev_max= std::max(std_dev_max,tmp);
	}
	std_dev = std::sqrt(std_dev/v.size());
	std_dev_max = std::sqrt(std_dev_max);
}

/////////////////////////////////////////////////////////////////////////////////////////////
int CurvatureSegmention::changeSegmentId(Facet_handle f, int new_id) {
	vector<int> result;
	int current_id=f->id;
	queue<Halfedge_handle> edgeQueue;

	int neigh_size=0;
	HF_circulator hh = f->facet_begin();
	f->id = new_id;
	neigh_size++;
	do {
		edgeQueue.push(hh);
	} while ( ++hh != f->facet_begin() );

	while ( !edgeQueue.empty() ) {

		Halfedge_handle edge = edgeQueue.front(); edgeQueue.pop();


		if (edge->opposite()->facet()->id==current_id) {
			edge->opposite()->facet()->id=new_id;
			neigh_size++;

			edgeQueue.push(edge->opposite()->next());
			edgeQueue.push(edge->opposite()->next()->next());
		}
	}
	return neigh_size;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CurvatureSegmention::segment(float threshold, int island_threshold) { // threshold = std_dev/mean
 // 1 - mean curvature	
#define NO_SEGM_CRITERIA 1
	float mean_values[NO_SEGM_CRITERIA]; 
	float std_dev_values[NO_SEGM_CRITERIA]; 
	float std_dev_max_values[NO_SEGM_CRITERIA];
	float tmp_value[NO_SEGM_CRITERIA];
	queue<Halfedge_handle> edgeQueue;
	vector<float> values[NO_SEGM_CRITERIA];
	
	cleanUp();
	float curv_bound_1, curv_bound_2;
	m->setVertexQuality(Mesh::Qual_Mean_Curv);
	m->computeQualityPercentileBoundaries(curv_bound_1,curv_bound_2,0.1);
	

	//reset the facet status
	for ( Facet_iterator i = m->p.facets_begin(); i != m->p.facets_end(); ++i )
	{
		i->removal_status = 'U';
		i->id=-1;
	}

	int no_components=0;
	//traverse the mesh via facets
	for ( Facet_iterator f = m->p.facets_begin(); f != m->p.facets_end(); ++f ) {
		bool new_seed_found = false;
		if ( f->removal_status=='U' ) { //test if it is a stable triangle
			//initialize
			for(int p=0;p<NO_SEGM_CRITERIA;p++) {
				values[p].clear();
				mean_values[p]=0;
				std_dev_values[p]=0;
			}
			HF_circulator h = f->facet_begin();
			do {
				getVertexStats(h,tmp_value,curv_bound_1,curv_bound_2);
				for(int p=0;p<NO_SEGM_CRITERIA;p++)
					values[p].push_back(tmp_value[p]);

			} while ( ++h != f->facet_begin() );

			for(int p=0;p<NO_SEGM_CRITERIA;p++)
				computeVectorStats(values[p],mean_values[p],std_dev_values[p],std_dev_max_values[p]);

	
			if ( (std_dev_max_values[0] < threshold) ) 
				new_seed_found = true;
			new_seed_found = true;
		}
		
		if (new_seed_found) {
			CurvatureSegment* curSegment = new CurvatureSegment();

			//create new segment
			curSegment->id=no_components++;
			curSegment->size=1;
			curSegment->facet=f;
			for ( int x=0;x<3;x++ ) curSegment->color[x]=rand() *1.0/RAND_MAX;

			int counter=values[0].size();
			f->removal_status='V'; // it is now visited
			f->id = curSegment->id;

			HF_circulator hh = f->facet_begin();
			do {
				edgeQueue.push(hh);
			} while ( ++hh != f->facet_begin() );
			
			while ( !edgeQueue.empty() ) {

				Halfedge_handle edge = edgeQueue.front(); edgeQueue.pop();

				if (edge->opposite()->facet()->removal_status=='U') {
					// check opposite vertex				
					getVertexStats(edge->opposite()->next(),tmp_value,curv_bound_1,curv_bound_2);

					for(int p=0;p<NO_SEGM_CRITERIA;p++)
						values[p].push_back(tmp_value[p]);

					for(int p=0;p<NO_SEGM_CRITERIA;p++)
						computeVectorStats(values[p],mean_values[p],std_dev_values[p],std_dev_max_values[p]);
		
					if ( std::abs(std_dev_max_values[0])< threshold ) { // it's a good triangle
						edgeQueue.push(edge->opposite()->next());
						edgeQueue.push(edge->opposite()->next()->next());
						
						edge->opposite()->facet()->removal_status='V';
						edge->opposite()->facet()->id = curSegment->id;
						curSegment->size++;
						int prev_size = values[0].size();
						counter++;
					}
					else {
						for(int p=0;p<NO_SEGM_CRITERIA;p++)
							values[p].pop_back();
					}
				}
				else {
					if (edge->opposite()->facet()->id != curSegment->id) {
						if (edge->opposite()->facet()->id != -1) {
							curSegment->addNeighbour(edge->opposite()->facet()->id);
							segments[edge->opposite()->facet()->id]->addNeighbour(curSegment->id);	
						}
						else {
							cout << "impossible" <<endl;
						}
					}

				}
			} //done traversing the current component
			segments.push_back(curSegment);
		} // found a new component
	}//done traversing the mesh
	cout << " - curvature based segmentation components: "<< no_components << endl;
	vector<CurvatureSegment*> sortedSegments = segments;
	std::sort(sortedSegments.begin(),sortedSegments.end(),CurvatureSegmentSortPredicate);

	/*
	for(int i=0;i<sortedSegments.size();i++)
		cout << sortedSegments[i]->size << " " ;
	cout << endl;

	for(int i=0;i<sortedSegments.size();i++) {
		cout << "-> " << i << " has neigh: ";
		for(int j=0;j<sortedSegments[i]->neigh.size();j++) {		
			cout << sortedSegments[i]->neigh[j] << " ";
		}
		cout << endl;
	}
	*/
		
	
	int processed=0;
	int not_processed=0;

	for(int i=0;i<sortedSegments.size();i++)
		if ((sortedSegments[i]->size<island_threshold) && (sortedSegments[i]->active)) {
			if (sortedSegments[i]->neigh.size()==0) {
				not_processed++;
			}
			else {
				processed++;
				int new_id = sortedSegments[i]->neigh[0];
				for(int j=0;j<sortedSegments[i]->neigh.size();j++) {
					if (segments[sortedSegments[i]->neigh[j]]->active && (segments[sortedSegments[i]->neigh[j]]->size > segments[new_id]->size))
						new_id = sortedSegments[i]->neigh[j];
				}
				int elements = changeSegmentId(sortedSegments[i]->facet,new_id);
				//cout << "changed " << elements << " elements for size=" << sortedSegments[i]->size << endl;
				int old_size=sortedSegments[i]->size;
				segments[new_id]->size+=sortedSegments[i]->size;
				if (segments[sortedSegments[i]->neigh[0]]->active==false)
					cout << "NOT GOOD!!!" << endl;

				for(int j=0;j<sortedSegments[i]->neigh.size();j++) {
					segments[sortedSegments[i]->neigh[j]]->deleteNeighbour(sortedSegments[i]->id);
					if (sortedSegments[i]->neigh[j]!=new_id) segments[sortedSegments[i]->neigh[j]]->addNeighbour(new_id);
				}
				sortedSegments[i]->size=10000000;
				sortedSegments[i]->active=false;
				sortedSegments[i]->id=-1;
				
			}
		}
	cout << "Small Patch Elimination: processed=" << processed << " and not_processed=" << not_processed << endl;

	/*
	sortedSegments = segments;
	std::sort(sortedSegments.begin(),sortedSegments.end(),CurvatureSegmentSortPredicate);
	for(int i=0;i<sortedSegments.size();i++)
		cout << sortedSegments[i]->size << " " ;
	cout << endl;
	 */

	//colouring
	if (colorFacets)
		for ( Halfedge_iterator e = m->p.halfedges_begin(); e != m->p.halfedges_end(); ++e ) {
			if (e->facet()->id==-1) {
				e->vertex()->set_color(1.0f,1.0f,1.0f);
			}
			else {
				float *color = segments[e->facet()->id]->color;
				e->vertex()->set_color(color[0],color[1],color[2]);
			}
		}
}
