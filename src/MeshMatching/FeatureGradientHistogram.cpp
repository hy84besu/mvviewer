#include "FeatureGradientHistogram.h"

#define GRADIENT_DEBUG(X) ;
//////////////////////////////////////////////////////////////////////////////////////////
FeatureGradientHistogram::FeatureGradientHistogram(int no_rings_, float avg_edge_size_, int no_bins_centroid_, int no_bins_groups_ , int no_bins_orientations_ , double spatial_influence_) {
	no_rings = no_rings_;
	avg_edge_size = avg_edge_size_;
	no_bins_orientations = no_bins_orientations_;
	no_bins_centroid = no_bins_centroid_;
	spatial_influence = spatial_influence_;
	no_bins_groups = no_bins_groups_;
}

Vector FeatureGradientHistogram::getGradient(Vertex& v) {
	return v.qual_vect;
}

//////////////////////////////////////////////////////////////////////////////////////////
Point FeatureGradientHistogram::rotatePointAroundVector(Vector axis, Point p, float a) {

  // taken from http://www.mines.edu/~gmurray/ArbitraryAxisRotation/ArbitraryAxisRotation.html

	float u,v,w,x,y,z, rx,ry,rz;

	u = axis.x();
	v = axis.y();
	w = axis.z();

	x=p.x();
	y=p.y();
	z=p.z();

	float norm2 = u*u+v*v+w*w;
	rx=(u*(u*x+v*y+w*z) + (x*(v*v + w*w) - u*(v*y+w*z))*cos(a)+sqrt(norm2)*(-w*y+v*z)*sin(a))/norm2;
	ry=(v*(u*x+v*y+w*z) + (y*(u*u + w*w) - v*(u*x+w*z))*cos(a)+sqrt(norm2)*( w*x-u*z)*sin(a))/norm2;
	rz=(w*(u*x+v*y+w*z) + (z*(v*v + u*u) - w*(v*y+u*x))*cos(a)+sqrt(norm2)*(-v*x+u*y)*sin(a))/norm2;

	return Point(rx,ry,rz);
}



//////////////////////////////////////////////////////////////////////////////////////////
float FeatureGradientHistogram::angleWithRefVector(Vector x, Vector origin, Vector normal) {
	Vector origin_rotated_90 = CGAL::cross_product(normal, origin);
	float angle = v_angle(x,origin);
	//float angle_2 = v_angle(x,origin_rotated_90);

	if (x*origin_rotated_90 < 0) angle = 2*PI-angle;
	angle = max(0.0f,min(angle,1.99f*PI));
	return angle;
}

void normalize_vector(float *v,int n) {
	float total=0.0f;
	for(int i=0;i<n;i++) total+=v[i]*v[i];
	total = sqrt(total);
	if (total>0.0f)
		for(int i=0;i<n;i++) v[i]/=total;
}


////////////////////////////////////////////////////////////////////////////////
// Given a value and a number of bins for a histogram, it computes the 3 indices
// as well as the 3 values where it amount has to be distributed.
////////////////////////////////////////////////////////////////////////////////
void  interpolate(float val, int no_bins, int bin_idx[3], float bin_vals[3]) {
//	float kernel[5] = {0.05f,0.15f,0.4f, 0.15f, 0.05f};
	int val_low = (int)floor(val);
	float val_middle = floor(val) + 0.5f;
	float ratio = val_middle - val;

	if (ratio>0) {
		bin_vals[0] = ratio;		
		bin_vals[1] = 1.0f-ratio;
		bin_vals[2] = 0.0f;
	}
	else {
		ratio = -ratio;
		bin_vals[0] = 0.0f;		
		bin_vals[1] = 1.0f-ratio;
		bin_vals[2] = ratio;
		
	}


	for(int i=0;i<3;i++) {
		bin_idx[i] = (val_low-1+i+no_bins)%no_bins;
	}
	
}


void FeatureGradientHistogram::computeFeatureVector(vector<float>& result, Vertex &v) {

#define FEATURE_VECT_DIM 3
	bool useInterp=true;
/*	
	//interpolation testing
 
	int bin_idx[3];	
	float bin_vals[3];

	float interp[] = {3.1f, 3.2f, 3.5f, 3.7f, 3.9f, 4.0f };
	for(int j=0;j<6;j++) {
		interpolate(interp[j], 10,bin_idx,bin_vals);
		printf("interpolation (%.2f) = [%.2f, %.2f %.2f]\n",interp[j],bin_vals[0],bin_vals[1],bin_vals[2]);
		
	}
*/
	result.clear();

	float maxGeodesic;
#define USE_GEODESIC_FEAT_COMP
#ifdef USE_GEODESIC_FEAT_COMP
	maxGeodesic = (no_rings+0.2)*avg_edge_size;
	vector<pair<Vertex*,float> > neighs = Mesh::getGeodesicNeighbourhood(v,maxGeodesic,false);	
	vector<pair<Vertex*,float> >::iterator n_it;
#else
	maxGeodesic = no_rings;
	vector<pair<Vertex*,int> > neighs = Mesh::getRingNeighbourhood(v,maxGeodesic,false);
	vector<pair<Vertex*,int> >::iterator n_it;	
#endif	
	

	int descriptor_size = no_bins_groups*no_bins_orientations*FEATURE_VECT_DIM;
	float descriptor[descriptor_size];

	

	Vector ref_normal = v.normal();
	Plane_3 ref_plane_point = Plane_3(v.point(),ref_normal);
	Vector ref_axis;

	// look for a neighbour that does not coincide with v
	HV_circulator vi = v.vertex_begin();
	if ( vi!=NULL )
	{
		do
		{
			ref_axis = ref_plane_point.projection(vi->opposite()->vertex()->point()) - v.point();
			ref_axis = v_normalized(ref_axis);
		}
		while ( ++vi != v.vertex_begin() && (v_norm(ref_axis)==0));
	}
	if (v_norm(ref_axis) == 0) return;


	//******************* COMPUTE LOCAL COORDINATE SYSTEM *******************
	
	//set the bins centroid vector
	vector<float> bins_centroid;
	bins_centroid.reserve(no_bins_centroid);
	for(int i=0;i<no_bins_centroid;i++) bins_centroid.push_back(0.0f);

	
	// compute the centroid information
	Vertex * neigh_v;
	float neigh_influence;
	GRADIENT_DEBUG(
				   cout << "====> id = " << v.id << " p=[" << v.point() <<  "]" << endl;
//				   cout << " - normal=[" << v.normal() <<  "]" << endl;
//				   cout << " - ref_axis=[" << ref_axis <<  "] from id=" << v.vertex_begin()->opposite()->vertex()->id << endl;				   
				   
				   )
	for(n_it=neighs.begin(); n_it!=neighs.end();n_it++) {
		neigh_v = n_it->first;
		neigh_influence = n_it->second  / maxGeodesic;
		

		Vector tmp_vec = neigh_v->point() - v.point();
		Vector tmp_vec_proj = ref_plane_point.projection(neigh_v->point()) - v.point();

		float tmp_angle = angleWithRefVector(tmp_vec_proj, ref_axis,ref_normal);
		float tmp_mag = v_norm(getGradient(*neigh_v))*GAUSSIAN((spatial_influence), neigh_influence);


		if (useInterp)
		{
			int bin_idx[3];	
			float bin_vals[3];
			
			interpolate(no_bins_centroid*tmp_angle/(2.0*PI), no_bins_centroid,bin_idx,bin_vals);
			
			for(int ii=0;ii<3;ii++)
				bins_centroid[bin_idx[ii]]+=bin_vals[ii]*tmp_mag;
			
		}
		else
		{
			int bin_index = floor(no_bins_centroid*tmp_angle/(2.0*PI)); // not used - replaced with interpolation

			if (!( (bin_index >=0) && (bin_index <no_bins_centroid))) {
				printf("bin_index=%d angle=%.2f\n",bin_index,tmp_angle);
				
			}
			assert( (bin_index >=0) && (bin_index <no_bins_centroid));
			
			//GRADIENT_DEBUG(cout << "centroid_index=" << bin_index << " angle=" << tmp_angle/(2*PI) << endl;)
			
			//bins_centroid[bin_index]+=tmp_mag;
			//bins_centroid[bin_index]+=tmp_mag*pow((float)v_norm(tmp_vec),(float)-spatial_influence);
			bins_centroid[bin_index]+=tmp_mag;			
			
			GRADIENT_DEBUG(
						   //					   printf(" - id=[%d] angle=[%.2f] magnitude=[%.2f] bin=[%d] \n",neigh_v->id, tmp_angle,tmp_mag,bin_index);
						   
						   //					   cout << "    - tmp_vec=[" << tmp_vec << "] of norm " << v_norm(tmp_vec) << endl;
						   //					   cout << "    - tmp_vec_proj=[" << tmp_vec_proj <<  "] of norm " << v_norm(tmp_vec_proj) << endl;					   
						   
						   )
			
		}
		
	}
	
	//find maximum centroid bin
	int max_index = 0; 
	for(int i=0;i<no_bins_centroid;i++)  {
		if (bins_centroid[i]>bins_centroid[max_index]) max_index = i;
	}

	GRADIENT_DEBUG(
//	cout << "centroid aligned ("<< neighs.size() << ")=";
//	for(int i=0;i<no_bins_centroid;i++) 
//		cout << bins_centroid[(max_index+i)%no_bins_centroid] << " ";
//	cout << endl;
	)

	ref_axis =  rotatePointAroundVector(ref_normal, Point(ref_axis.x(),ref_axis.y(),ref_axis.z()), 2.0*PI/no_bins_centroid*max_index) - Point(0,0,0);

	
	//******************* COMPUTE HISTOGRAM *******************
	//init
	for(int i=0;i<descriptor_size;i++) descriptor[i]=0.0f;

	Vector ref_normal_dim;
	Vector ref_axis_dim;	
	Plane_3 ref_plane_point_dim;

	
	for(int dim=0;dim<FEATURE_VECT_DIM;dim++) {
		if (dim==0) {
			ref_normal_dim=ref_normal;
			ref_axis_dim=ref_axis;
		}
		else if(dim==1) {
			ref_normal_dim = CGAL::cross_product(ref_normal,ref_axis);	
			ref_axis_dim=ref_axis;			
		}
		else {
			ref_normal_dim = ref_axis;
			ref_axis_dim=ref_normal;						
		}
		ref_plane_point_dim = Plane_3(v.point(),ref_normal_dim);
		
		// compute the histogram information - on the plane
		for(n_it=neighs.begin(); n_it!=neighs.end();n_it++) {
			neigh_v = n_it->first;
			neigh_influence = n_it->second  / maxGeodesic;

			Vector tmp_vec = neigh_v->point() - v.point();
			Vector tmp_vec_proj = ref_plane_point_dim.projection(neigh_v->point()) - v.point();
			
			float tmp_angle = angleWithRefVector(tmp_vec_proj, ref_axis_dim,ref_normal_dim);
			
			Vector grad = getGradient(*neigh_v);
			Vector grad_proj = ref_plane_point_dim.projection(v.point() + grad) - v.point();			

			float tmp_deriv_angle = angleWithRefVector(grad_proj, ref_axis_dim,ref_normal_dim);

			float tmp_mag = v_norm(grad)*GAUSSIAN((spatial_influence), neigh_influence);

			

			if (useInterp) 
			{
				int grp_bin_idx[3];	
				float grp_bin_vals[3];				
				interpolate(no_bins_groups*tmp_angle/(2.0*PI), no_bins_groups,grp_bin_idx,grp_bin_vals);				

				int ort_bin_idx[3];	
				float ort_bin_vals[3];
				interpolate(no_bins_orientations*tmp_deriv_angle/(2.0*PI), no_bins_orientations,ort_bin_idx,ort_bin_vals);

				for(int oo=0;oo<3;oo++)
					for(int gg=0;gg<3;gg++)			
						descriptor[(grp_bin_idx[gg] + no_bins_groups*dim)*no_bins_orientations+ort_bin_idx[oo]] += ort_bin_vals[oo]*grp_bin_vals[gg]*tmp_mag;
			}
			else 
			{
				int group_bin_index = floor(no_bins_groups*tmp_angle/(2.0*PI));
				group_bin_index=std::max(std::min(group_bin_index,no_bins_groups-1),0);		
				assert( (group_bin_index >=0) && (group_bin_index <no_bins_groups));
				
				int orient_bin_index = floor(no_bins_orientations*tmp_deriv_angle/(2.0*PI));
				orient_bin_index=std::max(std::min(orient_bin_index,no_bins_orientations-1),0);		
				assert( (orient_bin_index >=0) && (orient_bin_index < no_bins_orientations));
				
				//		GRADIENT_DEBUG(cout << "group_index=" << group_bin_index << " orientation_index=" << orient_bin_index;)
				//		GRADIENT_DEBUG(cout << " tmp_deriv_angle=" << tmp_deriv_angle;)
				//		GRADIENT_DEBUG(cout << endl;)
				
				int bin_index = (group_bin_index + no_bins_groups*dim)*no_bins_orientations+orient_bin_index;
				
				//			GRADIENT_DEBUG(printf("   - id=[%d] angle_space=[%.2f] angle_dir=[%.2f] magnitude=[%.2f] bin=[%d] \n",neigh_v->id, tmp_angle, tmp_deriv_angle,local_value,bin_index);)
				//			GRADIENT_DEBUG(printf("%d ",bin_index);)
				GRADIENT_DEBUG(printf("%.4f ",v_norm(grad_proj));)
				
				descriptor[bin_index]+= tmp_mag;				
				
			}
		}
		GRADIENT_DEBUG(printf("\n - ");)		
	}

/*	int dim_size = descriptor_size/FEATURE_VECT_DIM;
	for(int i=0;i<FEATURE_VECT_DIM;i++)
		normalize_vector(descriptor+dim_size*i,dim_size);
*/
	
	normalize_vector(descriptor,descriptor_size);

	//vector<float> result;
	result.reserve(descriptor_size);
	for(int i=0;i<descriptor_size;i++)
		result.push_back(descriptor[i]);

	GRADIENT_DEBUG(
				   printf("\n - DESC: ");
				   for(int i=0;i<descriptor_size;i++)
					printf("%.3f ",descriptor[i]);
				   printf("\n");
)
		
	
/*	if (v.id==34410) {
		cout << "done " << v.id << endl;
	}*/
//	return result;

}
