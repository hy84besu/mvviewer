/*
 *  PointsToMesh.cpp
 *  src
 *
 *  Created by Andrei Zaharescu on 12/07/07.
 *  Copyright 2007 __MyCompanyName__. All rights reserved.
 *
 */

#include "PointsToMesh.h"

#include <CGAL/Aff_transformation_3.h>
typedef CGAL::Aff_transformation_3<Kernel> Aff_trans;

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Destructeur
PointsToMesh::PointsToMesh(MVData* _data) : data(_data) {
    is_init = false;
    alg_saveOutput = false;
    alg_keepVerticesConstant = true;

	
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
// Destructeur
PointsToMesh::~PointsToMesh() {
    if (is_init)
        EndAlg();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// end of story
void PointsToMesh::EndAlg() {
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
// end of story
void PointsToMesh::setParams(const char *dst_points_file_, const char* src_mesh_file_, const char* src_vec_file_, bool reposition_, int init_remeshes_, double init_bbox_scale_, double dt_, int iters_, double scales_, double smoothing_, bool autoStop_, bool saveOutputs_, char* saveOutputPrefix_) {
	is_init = true;
	alg_init_remeshes = init_remeshes_;
	alg_init_bbox_scale = init_bbox_scale_;
	alg_dt = dt_;
	alg_scales = scales_;	
	alg_iters = iters_;
	alg_smoothing = smoothing_;
	alg_saveOutput = saveOutputs_;
	alg_autoStop = autoStop_;

	alg_reposition = reposition_;
	strcpy(alg_dstPointsFile,dst_points_file_);
	strcpy(alg_srcMeshFile,src_mesh_file_);
	strcpy(alg_srcVecFFile,src_vec_file_);	
	strcpy(alg_saveOutputPrefix,saveOutputPrefix_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// run a step

float PointsToMesh::RunStep() {
	cur_iter++;
	// COMPUTE THE MESH DISPLACEMENT

	for (Vertex_iterator vi = data->mesh.p.vertices_begin(); vi!=data->mesh.p.vertices_end(); vi++)
		vi->delta = Kernel::Vector_3(0,0,0);

// 1 closest 3d point to mesh point; 2 - closest mesh point to 3d point
#define EVOLUTION_METHOD 1
	
#if EVOLUTION_METHOD==1	
	// from current to the closest original
	for (Vertex_iterator vi = data->mesh.p.vertices_begin(); vi!=data->mesh.p.vertices_end(); vi++) {
		Kernel::Point_3 closest_point = dest_points->closestPoint(vi->point());
		Kernel::Vector_3 closest_normal = dest_points->normalMapping(closest_point);

/*		
		vi->delta = closest_point - vi->point();
					
		bool is_outside = v_norm((closest_point + closest_normal*v_norm(vi->delta)) - vi->point()) < v_norm(vi->delta);
//		bool is_outside = v_norm(v_normalized(closest_normal) + v_normalized(vi->delta)) < 1.4142;
//		bool is_outside = v_angle(closest_normal, vi->delta) > PI/2;		
		double dist_sign = (is_outside?1:-1);
		vi->delta = vi->normal()*v_norm(vi->delta)*(-1)*dist_sign;
*/
		
		vi->delta = vi->normal()* (closest_normal*(closest_point-vi->point()));
		
//		vi->delta == v_normalized(data->mesh.computeVectorComponent(vi->normal(),vi->delta,1))*v_norm(vi->delta);
//		vi->delta = v_normalized(vi->delta)*data->mesh.computeVertexStatistics(*vi,1)*0.05;
		//		vi->delta = vi->normal(); //*alg_dt
	}
#elif EVOLUTION_METHOD==2

	for(int i=0;i<dest_points->points.size();i++) {
			
		Kernel::Point_3 closest_point = dest_points->points[i]->point;
		Kernel::Vector_3 closest_normal = dest_points->points[i]->normal;	
		Kernel::Point_3 vertexPoint=data->mesh.closestPoint(dest_points->points[i]->point);
		Vertex* vi = data->mesh.vertexMapping(vertexPoint);
		if (vi!=NULL)
			vi->delta = vi->delta  + vi->normal()* (closest_normal*(closest_point-vi->point()));	
		//cout << "vi->delta=" << vi->delta << endl;
	}

#endif
	
	// NORMALIZE the movements
	float total_movement = 0;
	int total_elements = 0;
	double max_delta = data->mesh.edge_avg*alg_dt;
	for (Vertex_iterator vi = data->mesh.p.vertices_begin(); vi!=data->mesh.p.vertices_end(); vi++) {	
		//vi->delta = v_normalized(vi->delta)*data->mesh.computeVertexStatistics(*vi,1)*0.1;
//		double max_delta = data->mesh.computeVertexStatistics(*vi,1)*alg_dt;
//		double min_delta = data->mesh.edge_avg/5;

		
//		vi->delta = vi->delta;
//		vi->delta = vi->delta + v_normalized(vi->delta)*(RAND_MAX/2-std::rand())*1.0/RAND_MAX*data->mesh.edge_avg/2*alg_smoothing;
//		vi->delta = v_normalized(vi->delta)*max_delta;
		


		double the_norm = v_norm(vi->delta);		
		if (the_norm > max_delta) {
			 vi->delta = v_normalized(vi->delta)*max_delta;
		}
//		if (the_norm < min_delta) vi->delta = v_normalized(vi->delta)*min_delta;	
		the_norm = v_norm(vi->delta);
		if (the_norm > max_delta*0.5) {
			total_elements++;
			total_movement	+= v_norm(vi->delta);
		}
		
		//vi->delta = vi->delta + Kernel::Vector_3((RAND_MAX/2-std::rand())*1.0/RAND_MAX, (RAND_MAX/2-std::rand())*1.0/RAND_MAX, (RAND_MAX/2-std::rand())*1.0/RAND_MAX)*data->mesh.computeVertexStatistics(*vi,1)*alg_smoothing;
//		vi->delta = vi->delta + data->mesh.computeVectorComponent(vi->normal(),vi->laplacian()*alg_dt,0);		
		vi->delta = vi->delta + vi->laplacian()*alg_smoothing;	
	}


	// MOVE THE MESH
	OpenGLContext::mutex.lock();
	data->mesh.lock();
	for (Vertex_iterator vi = data->mesh.p.vertices_begin(); vi!=data->mesh.p.vertices_end(); vi++) {
		vi->prev_delta = vi->delta;
		vi->move ( vi->delta );
	}
	data->mesh.unlock();
	data->mesh.updateMeshData();
	OpenGLContext::mutex.unlock();
	
	//saveOutput
	if (alg_saveOutput) {
		char filename[300];
		sprintf(filename,"%s/output_%04d.off",alg_saveOutputPrefix,cur_iter);
		data->mesh.saveFormat(filename,"off");
		sprintf(filename,"%s/output_%04d.diff",alg_saveOutputPrefix,cur_iter);
		data->mesh.saveVectorField(filename);
		sprintf(filename,"%s/output_%04d.idx",alg_saveOutputPrefix,cur_iter);
		data->mesh.saveVertexIndices(filename);
		
	}
	
	emit stepFinished();
	return total_elements;
//	return (++cur_iter < iter);	

}
//////////////////////////////////////////////////////////////////////////////////////////////////////
// the algorithm
void PointsToMesh::run() {
	stopAlgPressed=false;
	cur_iter = 0;

	data->mesh.loadFormat(alg_srcMeshFile,false);
	dest_points = new PointSet(alg_dstPointsFile); //loading the points

	if (strlen(alg_srcVecFFile) > 2)
		data->mesh.loadVectorField(alg_srcVecFFile,false);

	if (alg_reposition) {
		float* bbox_dst=dest_points->getBoundingBox();
		float scale_dst= std::max(std::max((-bbox_dst[0]+bbox_dst[3])/2,(-bbox_dst[1]+bbox_dst[4])/2),(-bbox_dst[2]+bbox_dst[5])/2);
		float* bbox_src=data->mesh.getBoundingBox();
		float scale_src= std::max(std::max((-bbox_src[0]+bbox_src[3])/2,(-bbox_src[1]+bbox_src[4])/2),(-bbox_src[2]+bbox_src[5])/2);

		Kernel::Vector_3 rel_transl = Kernel::Point_3((bbox_dst[0]+bbox_dst[3])/2,(bbox_dst[1]+bbox_dst[4])/2,(bbox_dst[2]+bbox_dst[5])/2) - Kernel::Point_3((bbox_src[0]+bbox_src[3])/2,(bbox_src[1]+bbox_src[4])/2,(bbox_src[2]+bbox_src[5])/2);

		float rel_scale = alg_init_bbox_scale*scale_dst/scale_src;
		Aff_trans A = Aff_trans(CGAL::Translation(), rel_transl) * Aff_trans(CGAL::Scaling(), rel_scale );
		std::transform( data->mesh.p.points_begin(), data->mesh.p.points_end(), data->mesh.p.points_begin(), A);
	}

	data->mesh.saveVertexIndices("tmp.idx");
	
	//some remeshing of the 20 facet polygon
	for(int i=0;i<alg_init_remeshes;i++)
		data->mesh.remesh();
//	data->orig_mesh.computeMeshStats(); //compute the largest triangle

	double edge_min_orig = data->mesh.edge_avg*0.5;
	double edge_max_orig = data->mesh.edge_avg *2;

	for(int s=0;s<alg_scales;s++) {		
//		double edge_max = 1000000000;
		data->mesh.computeMeshStats(); //compute the largest triangle
		double edge_max = data->mesh.edge_avg *2;
		
		for(int t=0;t<alg_iters;t++) {
			data->mesh.computeMeshStats();
			cout.precision(8);

			OpenGLContext::mutex.lock();
			if (alg_keepVerticesConstant) 
				data->mesh.ensureEdgeSizes(edge_min_orig, edge_max_orig, 0.2, 170);
			else {
				//data->mesh.ensureEdgeSizes(data->orig_mesh.edge_min/2.1, edge_max, 0.3, 150, 10);
				cout << "make sure to fix this" <<endl;
			}
			data->mesh.removeSelfIntersections();
			OpenGLContext::mutex.unlock();

			float total_movement = RunStep();
			cout << "STEP (scale=" << s+1 << "/" << alg_scales <<", step=" << t+1 << "/" << alg_iters << ") moved=" << total_movement << endl;

			if ((total_movement==0.0) && (alg_autoStop==true)) {
				cout << "AutoStopping!" << endl;
				//stopAlgPressed = true;
				break;
			}
			if (stopAlgPressed) break;

		}
		if (stopAlgPressed) break;
		if (s!=0) {
			OpenGLContext::mutex.lock();
			data->mesh.remesh();
			OpenGLContext::mutex.unlock();
		}
	}
	//saveOutput
	if (alg_saveOutput) {
		char filename[300];
		sprintf(filename,"%s/output_final.off",alg_saveOutputPrefix);
		data->mesh.saveFormat(filename,"off");
		sprintf(filename,"%s/output_final.diff",alg_saveOutputPrefix);
		data->mesh.saveVectorField(filename);
		sprintf(filename,"%s/output_final.idx",alg_saveOutputPrefix);
		data->mesh.saveVertexIndices(filename);
		
	}
	cout << "End of evolution" << endl;
	EndAlg();	
}

