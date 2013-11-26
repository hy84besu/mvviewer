/*
 *  MVAlgVerif.cpp
 *  src
 *
 *  Created by Andrei Zaharescu on 12/07/07.
 *  Copyright 2007 __MyCompanyName__. All rights reserved.
 *
 */

#include "MVAlgVerif.h"
#include <CGAL/Aff_transformation_3.h>
#include <CGAL/Timer.h>
#include<stdlib.h>

typedef CGAL::Aff_transformation_3<Kernel> Aff_trans;

#define ADD_LENGTH_CONSTRAINT 0
#define MOVE_THE_MESH 1


//////////////////////////////////////////////////////////////////////////////////////////////////////
// Destructeur
MVAlgVerif::MVAlgVerif(MVData* _data) : data(_data) {
    is_init = false;
    alg_saveOutput = false;
    //alg_keepVerticesConstant = true;
    alg_keepVerticesConstant = false;

	
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
// Destructeur
MVAlgVerif::~MVAlgVerif() {
    if (is_init)
        EndAlg();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// end of story
void MVAlgVerif::EndAlg() {
	dst_mesh.vertex_mapping.clear();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
// end of story
void MVAlgVerif::setParams(const char* src_mesh_file_, const char* src_vec_file_, bool reposition_, int init_remeshes_, double init_bbox_scale_, int init_clones_, double dt_, int iters_, double scales_, double smoothing_, bool autoStop_, bool saveOutputs_, char* saveOutputPrefix_) {
	is_init = true;
	alg_init_remeshes = init_remeshes_;
	alg_init_bbox_scale = init_bbox_scale_;
	alg_init_clones = init_clones_;
	alg_dt = dt_;
	alg_scales = scales_;	
	alg_iters = iters_;
	alg_smoothing = smoothing_;
	alg_saveOutput = saveOutputs_;
	alg_autoStop = autoStop_;

	alg_reposition = reposition_;
	strcpy(alg_srcMeshFile,src_mesh_file_);
	strcpy(alg_srcVecFFile,src_vec_file_);	
	strcpy(alg_saveOutputPrefix,saveOutputPrefix_);
}

float MVAlgVerif::RunStep() {
	cur_iter++;
	// COMPUTE THE MESH DISPLACEMENT

	for (Vertex_iterator vi = data->mesh.p.vertices_begin(); vi!=data->mesh.p.vertices_end(); vi++) 
		vi->delta = Kernel::Vector_3(0,0,0);

	
	// from current to the closest destination point
	for (Vertex_iterator vi = data->mesh.p.vertices_begin(); vi!=data->mesh.p.vertices_end(); vi++) {
		Kernel::Point_3 closest_point = dst_mesh.closestPoint(vi->point());		
		//cerr << "closest_point (" << closest_point << ") has ID " << dst_mesh.vertex_mapping[closest_point]->id << endl;	
		Kernel::Vector_3 closest_normal = dst_mesh.vertex_mapping[closest_point]->normal();
		//Kernel::Vector_3 closest_normal = dst_mesh.vertexMapping(closest_point)->normal();

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
		if (vi->border()==false) vi->delta = vi->delta + vi->laplacian()*alg_smoothing;	
	}
	// MOVE THE MESH
	OpenGLContext::mutex.lock();
	data->mesh.lock();
	
	for (Vertex_iterator vi = data->mesh.p.vertices_begin(); vi!=data->mesh.p.vertices_end(); vi++) {
		if (alg_keepVerticesConstant) vi->delta = vi->normal()*(vi->delta*vi->normal());	
		vi->prev_delta = vi->delta;
		vi->border()=false;
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
// run a step
float MVAlgVerif::RunColorStep() {
	cur_iter++;
	cerr << "ITERATION ( " << cur_iter << ") " << endl;
	// COMPUTE THE MESH DISPLACEMENT

	for (Vertex_iterator vi = data->mesh.p.vertices_begin(); vi!=data->mesh.p.vertices_end(); vi++) {
		vi->delta = Kernel::Vector_3(0,0,0);
		vi->delta_tmp = Kernel::Vector_3(0,0,0);
	}

	
	// from current to the closest destination point
	for (Vertex_iterator vi = data->mesh.p.vertices_begin(); vi!=data->mesh.p.vertices_end(); vi++) {

#if MOVE_THE_MESH
		Kernel::Point_3 closest_point = dst_mesh.closestColorPoint(vi->point(), (float *)(vi->color));	
		//Kernel::Point_3 closest_point = dst_mesh.closestPoint(vi->point());	
#else 
		Kernel::Point_3 tmp_point = vi->point() + vi->motion_vec;
		Kernel::Point_3 closest_point = dst_mesh.closestColorPoint(tmp_point,  (float *)(vi->color));	
#endif
		//cerr << "closest_point (" << closest_point << ") has ID " << dst_mesh.vertex_mapping[closest_point]->id << endl;	
		//Kernel::Vector_3 closest_normal = dst_mesh.vertex_mapping[closest_point]->normal();
		//Kernel::Vector_3 closest_normal = dst_mesh.vertexMapping(closest_point)->normal();

/*		
		vi->delta = closest_point - vi->point();
					
		bool is_outside = v_norm((closest_point + closest_normal*v_norm(vi->delta)) - vi->point()) < v_norm(vi->delta);
//		bool is_outside = v_norm(v_normalized(closest_normal) + v_normalized(vi->delta)) < 1.4142;
//		bool is_outside = v_angle(closest_normal, vi->delta) > PI/2;		
		double dist_sign = (is_outside?1:-1);
		vi->delta = vi->normal()*v_norm(vi->delta)*(-1)*dist_sign;
*/
		
		vi->delta = closest_point- (vi->point() + vi->motion_vec); 
 
		//vi->delta = vi->normal()* (closest_normal*(closest_point-vi->point())); 

//		vi->delta == v_normalized(data->mesh.computeVectorComponent(vi->normal(),vi->delta,1))*v_norm(vi->delta);
//		vi->delta = v_normalized(vi->delta)*data->mesh.computeVertexStatistics(*vi,1)*0.05;
		//		vi->delta = vi->normal(); //*alg_dt
	}

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

		if (! MOVE_THE_MESH || the_norm > max_delta*0.5) {
			total_elements++;
			total_movement	+= v_norm(vi->delta);
		}
		
		//vi->delta = vi->delta + Kernel::Vector_3((RAND_MAX/2-std::rand())*1.0/RAND_MAX, (RAND_MAX/2-std::rand())*1.0/RAND_MAX, (RAND_MAX/2-std::rand())*1.0/RAND_MAX)*data->mesh.computeVertexStatistics(*vi,1)*alg_smoothing;
//		vi->delta = vi->delta + data->mesh.computeVectorComponent(vi->normal(),vi->laplacian()*alg_dt,0);		
		//vi->delta = vi->delta + vi->laplacian()*alg_smoothing;	
	}
	data->mesh.diffuse(alg_smoothing, 0); 
#if ADD_LENGTH_CONSTRAINT
	for (Vertex_iterator vi = data->mesh.p.vertices_begin(); vi!=data->mesh.p.vertices_end(); vi++) {	
	// Add a cost for preserving the smoothness and the geometry of the mesh
	       HV_circulator h = vi->vertex_begin();
	       double geom_vx = 0 ;
	       double geom_vy = 0 ;
	       double geom_vz = 0 ;   
	       int order=0;

	       do
	       {
	           const float xx = TO_FLOAT ( vi->point().x() );
	           const float yy = TO_FLOAT ( vi->point().y() );
        	   const float zz = TO_FLOAT ( vi->point().z() );
                   const float xxx = TO_FLOAT ( h->opposite()->vertex()->point().x()) ;
        	   const float yyy = TO_FLOAT ( h->opposite()->vertex()->point().y()) ;
	           const float zzz = TO_FLOAT ( h->opposite()->vertex()->point().z()) ;

        	   double d = (xx - xxx) * (xx - xxx)  + (yy - yyy) * (yy - yyy) + (zz - zzz) * (zz - zzz) ;
#if MOVE_THE_MESH
	           double t_d2x = (xx + vi->delta[0]) - (xxx + h->opposite()->vertex()->delta[0]) ;
        	   double t_d2y = (yy + vi->delta[1]) - (yyy + h->opposite()->vertex()->delta[1]) ;
	           double t_d2z = (zz + vi->delta[2]) - (zzz + h->opposite()->vertex()->delta[2]) ;
#else
	           double t_d2x = (xx + vi->motion_vec[0] + vi->delta[0]) - (xxx + h->opposite()->vertex()->motion_vec[0] + h->opposite()->vertex()->delta[0]) ;
        	   double t_d2y = (yy + vi->motion_vec[1] + vi->delta[1]) - (yyy + h->opposite()->vertex()->motion_vec[1] + h->opposite()->vertex()->delta[1]) ;
	           double t_d2z = (zz + vi->motion_vec[2] + vi->delta[2]) - (zzz + h->opposite()->vertex()->motion_vec[2] + h->opposite()->vertex()->delta[2]) ;
#endif
	           double d2 = t_d2x * t_d2x + t_d2y * t_d2y + t_d2z * t_d2z;
                   geom_vx += t_d2x * (sqrt(d2) - sqrt(d)) / sqrt(d2) ;
        	   geom_vy += t_d2y * (sqrt(d2) - sqrt(d)) / sqrt(d2) ;
	           geom_vz += t_d2z * (sqrt(d2) - sqrt(d)) / sqrt(d2) ;
        	   order++;

	       }
	       while ( ++h != vi->vertex_begin() );
	       geom_vx /= order ;
	       geom_vy /= order ;
	       geom_vz /= order ;
	       double alpha = 1 ;
	       vi->delta_tmp =  alpha * Vector(-geom_vx, -geom_vy, -geom_vz) ; 
#if 0
	if(v_norm(vi->delta_tmp) > 0.001)  
	       std::cout << vi->delta << " +  " << geom_vx << " " << geom_vy << " " << geom_vz <<std::endl ;
#endif
	}
	for (Vertex_iterator vi = data->mesh.p.vertices_begin(); vi!=data->mesh.p.vertices_end(); vi++) {	
	       double alpha1 = 0.9, alpha2 = 0.1 ;
		if(v_norm(vi->delta)<0.01) { alpha2 = 0.9; alpha1 = 0.1; }
		vi->delta = alpha1 * vi->delta + alpha2 * vi->delta_tmp;
	}
#endif 
	// MOVE THE MESH
	OpenGLContext::mutex.lock();
	data->mesh.lock();
	for (Vertex_iterator vi = data->mesh.p.vertices_begin(); vi!=data->mesh.p.vertices_end(); vi++) {
		if (alg_keepVerticesConstant) vi->delta = vi->normal()*(vi->delta*vi->normal());	
		vi->prev_delta = vi->delta;
#if MOVE_THE_MESH
		vi->move ( vi->delta );
#else
		vi->motion_vec = vi->motion_vec + vi->delta;
#endif
	}
	data->mesh.unlock();
#if MOVE_THE_MESH
	data->mesh.updateMeshData();
#else
	//for (Vertex_iterator vi = data->mesh.p.vertices_begin(); vi!=data->mesh.p.vertices_end(); vi++) {
	//	vi->motion_vec = vi->motion_vec + vi->laplacian() * alg_smoothing; // smooth the motion vectors
	//}
#endif
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
	return total_movement;
	//return total_elements;
//	return (++cur_iter < iter);	

}

void MVAlgVerif::selectClosest_tmp() { 
	int i=0;
	FILE *select = fopen("select.dat", "w");
	Vertex_handle vdst;	
	for (Vertex_iterator vi = dst_mesh.p.vertices_begin(); vi != dst_mesh.p.vertices_end(); vi++) {
	vi->id = -1;
	}

	for (Vertex_iterator vi = data->mesh.p.vertices_begin(); vi!=data->mesh.p.vertices_end(); vi++) {
		Kernel::Point_3 closest_point = dst_mesh.closestPoint(vi->point());
		cerr << i << ": " << vi->point() << " closest to " << closest_point << endl; 
		if(v_norm(closest_point - vi->point()) < dst_mesh.edge_avg*alg_smoothing) {
			vdst = dst_mesh.vertex_mapping[closest_point]; 
			cerr << "whose id is " << vdst->id; 
			vdst->weight = 2;
			vdst->id = i; 
			cerr << " changed to " << vdst->id << endl;
		} i++;
	}
	i=0;
	for (Vertex_iterator vi = dst_mesh.p.vertices_begin(); vi != dst_mesh.p.vertices_end(); vi++) {
		if(vi->weight == 2) {
			fprintf(select, "%d\t%d\n", i,vi->id);
		}
		i++;
	}
	fclose(select);
	cerr << "Done with printing out stuff ! Exiting " << endl;
	exit(0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// the algorithm
void MVAlgVerif::run() {
	CGAL::Timer time_elapsed;
	CGAL::Timer transformesh_elapsed;	
	
	stopAlgPressed=false;
	cur_iter = 0;

	dst_mesh.loadFormat(data->mesh.loaded_filename,true);
	cerr << "Loaded destination-mesh from file : " << data->mesh.loaded_filename << endl;
	// Populate the point mapping : will be done implicitly
	//dst_mesh.vertexMapping( dst_mesh.p.vertices_begin()->point() );

	data->mesh.loadFormat(alg_srcMeshFile,false);
	cerr << "Loaded init-mesh from file : " << alg_srcMeshFile << endl;

	// selectClosest_tmp(); // a temporary hack for selecting closest points

	if (strlen(alg_srcVecFFile) > 2)
		data->mesh.loadVectorField(alg_srcVecFFile,false);


	if (alg_reposition) {
		float* bbox=dst_mesh.getBoundingBox();
		// Apply affine transformation to obtain initial configuration
/*		Kernel::Vector_3 rel_transl((bbox[0]+bbox[3])/2,(bbox[1]+bbox[4])/2,(bbox[2]+bbox[5])/2);
		float rel_scale = alg_init_bbox_scale*std::max(std::max((-bbox[0]+bbox[3])/2,(-bbox[1]+bbox[4])/2),(-bbox[2]+bbox[5])/2);
		Aff_trans A = Aff_trans(CGAL::Translation(), rel_transl) * Aff_trans(CGAL::Scaling(), rel_scale );
		std::transform( data->mesh.p.points_begin(), data->mesh.p.points_end(), data->mesh.p.points_begin(), A);
*/

/*		Mesh templateMesh=data->mesh;
		Mesh tmpMesh=templateMesh;
		tmpMesh=templateMesh;
*/
		//cout << "orig mesh" << NULL << ";" << data->mesh.loaded_filename << ";"  << data->mesh.loaded_file_type << ";" << data->mesh.loaded_vect_filename << ";" << endl;
		cout.flush();
		Mesh templateMesh=data->mesh;
		data->mesh.clear(false);
		int divs=alg_init_clones;;
		float rel_scale_2 = alg_init_bbox_scale*std::max(std::max((-bbox[0]+bbox[3]),(-bbox[1]+bbox[4])),(-bbox[2]+bbox[5]))/(2.1*divs);

		Mesh tmpMesh;
		for(int iz=0; iz<divs;iz++)
			for(int iy=0; iy<divs;iy++)
				for(int ix=0; ix<divs;ix++) {
					tmpMesh=templateMesh;
					Kernel::Vector_3 rel_transl_2(bbox[0]+abs(-bbox[0]+bbox[3])*(ix+0.5)/divs,bbox[1]+ abs(-bbox[1]+bbox[4])*(iy+0.5)/divs,bbox[2]+abs(-bbox[2]+bbox[5])*(iz+0.5)/divs);
					Aff_trans B = Aff_trans(CGAL::Translation(), rel_transl_2) * Aff_trans(CGAL::Scaling(), rel_scale_2 );
					std::transform( tmpMesh.p.points_begin(), tmpMesh.p.points_end(), tmpMesh.p.points_begin(), B);
					data->mesh.unionWith(tmpMesh);
					cout << "processing " << ix << " "  << iy << " " << iz << " -> translating to \t" << rel_transl_2 << endl;
				}
//		cout << "tmpMesh " << NULL << " " << tmpMesh.loaded_filename << ";"  << tmpMesh.loaded_file_type << " " << tmpMesh.loaded_vect_filename << endl;

	}


	data->mesh.saveVertexIndices("tmp.idx");

#if MOVE_THE_MESH	
	//some remeshing of the 20 facet polygon
	for(int i=0;i<alg_init_remeshes;i++)
		data->mesh.remesh();
	dst_mesh.computeMeshStats(); //compute the largest triangle
#else
	data->mesh.updateMeshData();
#endif

	data->mesh.saveFormat("test.off","off");

	double edge_min_orig = data->mesh.edge_avg*0.5;
	double edge_max_orig = data->mesh.edge_avg *2;

	time_elapsed.start();
	
	int stats_intersections=0;
	int stats_iterations=0;
	int stats_no_facets=0;
	double stats_transformesh_time=0;	
	
	for(int s=0;s<alg_scales;s++) {		
//		double edge_max = 1000000000;
		data->mesh.computeMeshStats(); //compute the largest triangle
		double edge_max = data->mesh.edge_avg *2;
		
		for(int t=0;t<alg_iters;t++) {

#if MOVE_THE_MESH
			OpenGLContext::mutex.lock();
			if (alg_keepVerticesConstant) 
				data->mesh.ensureEdgeSizes(edge_min_orig, edge_max_orig, 0.2, 160,10, 2);
			else
				data->mesh.ensureEdgeSizes(dst_mesh.edge_min/2.1, edge_max, 0.3, 150, 10, 2);
			transformesh_elapsed.start();
			data->mesh.removeSelfIntersections();
			stats_transformesh_time+=transformesh_elapsed.time();
			OpenGLContext::mutex.unlock();
#endif
			//float total_movement = RunColorStep();
			float total_movement = RunStep();
			cout << "STEP (scale=" << s+1 << "/" << alg_scales <<", step=" << t+1 << "/" << alg_iters << ") moved=" << total_movement << endl;


			if ((total_movement==0.0) && (alg_autoStop==true)) {
				//compute Housdorf distance
				float max_dist_1=data->mesh.distanceTo(dst_mesh,true);
				float max_dist_2=dst_mesh.distanceTo(data->mesh,false);
				float max_dist=std::max(max_dist_1,max_dist_2);
				if(max_dist>2*data->mesh.edge_avg) {
					cout << "No movement but still far from the solution. d=" << max_dist << " edge_avg=" << data->mesh.edge_avg << endl;

				}
				else {
					cout << "AutoStopping!" << endl;
					stopAlgPressed = true;
				}
			}
			if (stopAlgPressed) break;

			data->mesh.close_color_sigma *= data->mesh.close_color_sigma; 
			//data->mesh.close_color_sigma /= sqrt(2); 
//			cerr << "close_color_sigma " << data->mesh.close_color_sigma << endl;
			data->mesh.computeMeshStats();
			cout.precision(8);
			
			stats_intersections+=data->mesh.no_intersections;
			stats_iterations++;
			stats_no_facets+=data->mesh.p.size_of_facets();;
			
		}
		if (stopAlgPressed) break;
#if MOVE_THE_MESH
		if (s!=0) {
			OpenGLContext::mutex.lock();
			data->mesh.remesh();
			OpenGLContext::mutex.unlock();
		}
#endif
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
	cout << "Total Running Time:  "  << ( ( int ) time_elapsed.time() ) /60 << ":" << ( ( int ) time_elapsed.time() ) %60 << endl;
	cout << "-----------------------------------------------------------------------------" << endl;
	cout << "Statistics:" << endl;
	cout << " - iterations: " << stats_iterations << endl;
	if (stats_iterations>0) {
		cout << " - avg. no. intersections: " << stats_intersections*1.0/stats_iterations << endl;
		cout << " - avg. no. facets: " << stats_no_facets*1.0/stats_iterations << endl;
		cout << " - avg. time alg (sec): " << time_elapsed.time()*1.0/stats_iterations << endl;	
		cout << " - avg. time TransforMesh (sec): " << stats_transformesh_time*1.0/stats_iterations << endl;			
	}

	
	EndAlg();	
}

