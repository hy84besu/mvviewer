#include "FeatureGradientHistogram.h"

#define GRADIENT_DEBUG(X) ;
//////////////////////////////////////////////////////////////////////////////////////////
FeatureGradientHistogram::FeatureGradientHistogram(int feature_type_, int no_rings_, int no_bins_centroid_, int no_bins_groups_ , int no_bins_orientations_ , double spatial_influence_) {
	no_rings = no_rings_;
	no_bins_orientations = no_bins_orientations_;
	no_bins_centroid = no_bins_centroid_;
	spatial_influence = spatial_influence_;
	no_bins_groups = no_bins_groups_;
	feature_type = feature_type_;
	assert((feature_type>=0) && (feature_type<=2));
}

Vector FeatureGradientHistogram::getGradient(Vertex& v, int feat_type) {
	return v.qual_vect;
	
	if (feat_type==0) return Mesh::computeVertexGradient(v,Mesh::Qual_Color_Deriv);
	if (feat_type==1) return Mesh::computeVertexGradient(v,Mesh::Qual_Mean_Curv_Deriv);
	if (feat_type==2) return Mesh::computeVertexGradient(v,Mesh::Qual_Gaussian_Curv_Deriv);
	if (feat_type>2) {
		cout << "uknown feature type in getGradient FeatureGradientHistogram!!!" << endl;
	}

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
	return angle;
}

void normalize_vector(float *v,int n) {
	float total=0.0f;
	for(int i=0;i<n;i++) total+=v[i];
	if (total>0.0f)
		for(int i=0;i<n;i++) v[i]/=total;
}

void FeatureGradientHistogram::computeFeatureVector(vector<float>& result, Vertex &v) {
	assert(feature_type<=2);
	computeFeatureVector(result,v,feature_type);
	if (feature_type>2) {
/*		cout << "Invalid feature type!!! in FeatureGradientHistogram::computeFeatureVector" <<endl;
		vector<float> r2;
		computeFeatureVector(result,v,0);
		computeFeatureVector(r2,v,1);
		for(int i=0;i<r2.size();i++) result.push_back(r2[i]);*/
	}
	else
		computeFeatureVector(result,v,feature_type);
}

void FeatureGradientHistogram::computeFeatureVector(vector<float>& result, Vertex &v, int feat_type) {

#define FEATURE_VECT_DIM 2

	result.clear();

	vector<pair<Vertex*,int> > neighs = Mesh::getNeighbourhood(v,no_rings);
	vector<pair<Vertex*,int> >::iterator n_it;
	vector<float> bins_centroid;

	//set the bins centroid vector
	bins_centroid.reserve(no_bins_centroid);
	for(int i=0;i<no_bins_centroid;i++) bins_centroid.push_back(0.0f);

	int descriptor_size = no_bins_groups*no_bins_orientations*FEATURE_VECT_DIM;
	float descriptor[descriptor_size];


	Vector ref_normal = v.normal();
	Plane_3 ref_plane_point = Plane_3(v.point(),ref_normal);
	Plane_3 ref_plane_zero = Plane_3(Point(0.0,0.0,0.0),ref_normal);
	Vector ref_axis = ref_plane_zero.projection(v.vertex_begin()->prev()->vertex()->point())-Point(0.0,0.0,0.0);//ref_plane_zero.base1();

	//descriptor.reserve(descriptor_size);
	//for(int i=0;i<descriptor_size;i++) descriptor.push_back(0.0f);

	// compute the centroid information
	Vertex * neigh_v;
	int neigh_v_ring;
	for(n_it=neighs.begin(); n_it!=neighs.end();n_it++) {
		neigh_v = n_it->first;
		neigh_v_ring = n_it->second;
		float tmp_mag = v_norm(getGradient(*neigh_v,feat_type));
		Vector tmp_vec = neigh_v->point() - v.point();
		Vector tmp_vec_proj = ref_plane_point.projection(neigh_v->point()) - v.point();

		float tmp_angle = angleWithRefVector(tmp_vec_proj, ref_axis,ref_normal);

		int bin_index = floor(tmp_angle/(2.0*PI/no_bins_centroid));
		assert( (bin_index >=0) && (bin_index <no_bins_centroid));
		//GRADIENT_DEBUG(cout << "centroid_index=" << bin_index << " angle=" << tmp_angle/(2*PI) << endl;)

		//bins_centroid[bin_index]+=tmp_mag;
		//bins_centroid[bin_index]+=tmp_mag*pow((float)v_norm(tmp_vec),(float)-spatial_influence);
		bins_centroid[bin_index]+=tmp_mag*GAUSSIAN((1.0f-spatial_influence), (neigh_v_ring*1.0f/no_rings));
	}

	//find maximum centroid bin
	int max_index = 0; 
	for(int i=0;i<no_bins_centroid;i++)  {
		if (bins_centroid[i]>bins_centroid[max_index]) max_index = i;
	}

	GRADIENT_DEBUG(
	cout << "centroid aligned ("<< neighs.size() << ")=";
	for(int i=0;i<no_bins_centroid;i++) 
		cout << bins_centroid[(max_index+i)%no_bins_centroid] << " ";
	cout << endl;
	)

	ref_axis =  rotatePointAroundVector(ref_normal, Point(ref_axis.x(),ref_axis.y(),ref_axis.z()), 2.0*PI/no_bins_centroid*max_index) - Point(0,0,0);

	//init
	for(int i=0;i<descriptor_size;i++) descriptor[i]=0.0f;

	Vector ref_normal_dim;
	Plane_3 ref_plane_point_dim;
	Plane_3 ref_plane_zero_dim;
	
	for(int dim=0;dim<FEATURE_VECT_DIM;dim++) {
		if (dim==0) {
			ref_normal_dim=ref_normal;
			ref_plane_point_dim=ref_plane_point;
			ref_plane_zero_dim=ref_plane_zero;			
		}
		else if(dim==1) {
			ref_normal_dim = CGAL::cross_product(ref_normal,ref_axis);	
			ref_plane_point_dim = Plane_3(v.point(),ref_normal_dim);
			ref_plane_zero_dim = Plane_3(Point(0,0,0),ref_normal_dim);	
			
		}
		else {
			ref_normal_dim = CGAL::cross_product(CGAL::cross_product(ref_normal,ref_axis),ref_axis);	
			ref_plane_point_dim = Plane_3(v.point(),ref_normal_dim);
			ref_plane_zero_dim = Plane_3(Point(0,0,0),ref_normal_dim);				
		}
		
		// compute the histogram information - on the plane
		for(n_it=neighs.begin(); n_it!=neighs.end();n_it++) {
			neigh_v = n_it->first;
			neigh_v_ring = n_it->second;
			Vector tmp_vec = neigh_v->point() - v.point();
			Vector tmp_vec_proj = ref_plane_point_dim.projection(neigh_v->point()) - v.point();

			float tmp_angle = angleWithRefVector(tmp_vec_proj, ref_axis,ref_normal_dim);
			int group_bin_index = floor(tmp_angle/(2.0*PI/no_bins_groups));
			group_bin_index=std::max(std::min(group_bin_index,no_bins_groups-1),0);		
			assert( (group_bin_index >=0) && (group_bin_index <no_bins_groups));
			
			Vector grad = getGradient(*neigh_v,feat_type);
			Vector grad_proj = ref_plane_zero_dim.projection(Point(grad.x(), grad.y(), grad.z())) - Point(0,0,0);


			float tmp_deriv_angle = angleWithRefVector(grad_proj, ref_axis,ref_normal_dim);
			int orient_bin_index = floor(tmp_deriv_angle/(2.0*PI/no_bins_orientations));
			orient_bin_index=std::max(std::min(orient_bin_index,no_bins_orientations-1),0);		
			assert( (orient_bin_index >=0) && (orient_bin_index < no_bins_orientations));

	//		GRADIENT_DEBUG(cout << "group_index=" << group_bin_index << " orientation_index=" << orient_bin_index;)
	//		GRADIENT_DEBUG(cout << " tmp_deriv_angle=" << tmp_deriv_angle;)
	//		GRADIENT_DEBUG(cout << endl;)
			float local_value;
			local_value = v_norm(grad_proj)*GAUSSIAN((1.0f-spatial_influence), (neigh_v_ring*1.0f/no_rings));
			//cout << GAUSSIAN((1.0f-spatial_influence), (neigh_v_ring*1.0f/no_rings)) << endl;
			//local_value = v_norm(grad_proj);
			//local_value = 1;

			//descriptor[group_bin_index*no_bins_orientations+orient_bin_index]++;
			//descriptor[group_bin_index*no_bins_orientations+orient_bin_index]+= pow((float)v_norm(tmp_vec_proj),-spatial_influence)*v_norm(grad);
			//descriptor[group_bin_index*no_bins_orientations+orient_bin_index]+= pow((double)v_norm(tmp_vec),-spatial_influence);

			//descriptor[group_bin_index*no_bins_orientations+orient_bin_index]+= local_value;
			descriptor[(group_bin_index + no_bins_groups*dim)*no_bins_orientations+orient_bin_index]+= local_value;			
		}
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

/*	if (v.id==34410) {
		cout << "done " << v.id << endl;
	}*/
//	return result;

}
