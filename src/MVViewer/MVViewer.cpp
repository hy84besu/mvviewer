/*
 *  MVViewer.cpp
 *  src
 *
 *  Created by Andrei Zaharescu on 28/11/06.
 *  Copyright 2006 __MyCompanyName__. All rights reserved.
 *
 */

#include "MVViewer.h"
#include "Vector3.h"

#include <stdlib.h>
#include <string.h>


/*********************************************************************************/
MVViewer::MVViewer(QWidget *parent):QGLViewer(parent) {
	geom_enum = GL_TRIANGLES;
	
	texturing_ = true;
	cameras_ = false;
	lighting_ = true;
	flatShading_ = false;
	bounding_box_ = false;
	normals_ = false;
	deltas_ = false;
	curvature_ = false;
	edges_ = false;
	textures_=false;

	segment_intersections_=false;
	segment_delaunay_=false;
	texturing_ = false;
	debug_info_ = false;
	debug_info_one_ = false;
	motion_field_ = false;

	bg_color[0]=0.0; bg_color[1]=0.0; bg_color[2]=0.0;
	
	data = NULL;
	gl = NULL; 
	glc = new OpenGLContext(this);
	
	transformesh_colors_max_facet_no=-1;	
	geometry_mode = 'N'; //just normal white triangles
	quality_mode = Mesh::Qual_Mean_Curv;
	
	setScalingFactor(0.003);
	polygon_draw_mode='F';
	resize(640,480);
}



void MVViewer::paintGL() {
	glc->makeCurrent();
	QGLViewer::paintGL();
	glc->doneCurrent();
}

void MVViewer::updateGL() {
	glc->makeCurrent();
	QGLViewer::updateGL();
	glc->doneCurrent();
}

void MVViewer::resizeGL(int width, int height){
	glc->makeCurrent();
	QGLViewer::resizeGL(width,height);
	glc->doneCurrent();
}

void MVViewer::initializeGL () {
	glc->makeCurrent();
	QGLViewer::initializeGL();
	glc->doneCurrent();
}


/*
void MVViewer::makeCurrent() {
	glc->makeCurrent();
}
*/

MVViewer::~MVViewer(){
	if (gl) delete gl;
	if (glc) delete glc;
}

void MVViewer::setScalingFactor(double newScale) {
	scaling_factor = newScale;
	if (gl) gl->sc = scaling_factor;
}


void MVViewer::downScaleInputs(int scale) {

	data->downScaleInputs(scale);
	setMVData(data);
	init();
}


void MVViewer::setMVData(MVData *_data) {
	data = _data;
	if (gl) delete gl;
//	if (glc) delete glc;

//	glc = new OpenGLContext(this);
	gl = new MVViewerRendering(glc,data,data->imageMaxWidth(),data->imageMaxWidth());
	gl->sc = scaling_factor;

	// initialize from the data matrix
	glc->makeCurrent();
	for(int i=0;i<data->ncameras;i++) {
		gl->InitCamera(i,data->cameras[i],data->mesh.getBoundingBox());
		if (textures_) gl->InitTexture(i,data->images[i]);
	}
	glc->doneCurrent();
}

/*********************************************************************************/
void MVViewer::draw() {
	Point render_point;	
	if (data==NULL) return;
	/*
	 CImg<> reproj(data->W,data->H,1,data->is_rgb?3:1);
	 CImg<unsigned char> mask(data->W,data->H);
	 
	 gl->RenderTexture(1,2,reproj,mask);
	 cimg_forXY(reproj,x,y) if ((reproj(x,y,0,0) != 0 || reproj(x,y,0,1) !=0 || reproj(x,y,0,2)!=0) && !mask(x,y)) { reproj(x,y,0,0) = 255; reproj(x,y,0,1) = 255; reproj(x,y,0,2) = 0; }
	 mask.save("mask.pgm");
	 reproj.save("reproj.ppm");
	 */

/*	
	CImg<> reproj(data->W,data->H,1,data->is_rgb?3:1);
    CImg<unsigned char> mask(data->W,data->H);
*/
//	glc->makeCurrent();
//	setBackgroundColor(QColor(255,255,0));
//	setBackgroundColor(QColor(255,255,255));	
	glClearColor(bg_color[0],bg_color[1],bg_color[2],0);
	
	// Save the current model view matrix (not needed here in fact)
	glPushMatrix();
	// Multiply matrix to get in the frame coordinate system.
	glMultMatrixd(manipulatedFrame()->matrix());
	glShadeModel(GL_SMOOTH);	

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//	glDepthRange (0.1, 1.0); /* Draw underlying geometry */ 

	if (cameras_) gl->DrawCameras(sceneRadius()/2.);

	glLineWidth(1);
	if (polygon_draw_mode=='P') glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
	if (polygon_draw_mode=='W') glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	if (polygon_draw_mode=='F') glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	data->mesh.lock();
 	glEnable(GL_POLYGON_OFFSET_FILL);
   	glPolygonOffset(1.0, 1.0);
	if (texturing_) gl->DrawGeometryWithTextures();
	else gl->DrawGeometry(geom_enum,geometry_mode);

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
   	glPolygonOffset(0.5, 1.0);
	if (debug_info_) { //show all vertex ids + facet ids
		int counter=0;
		glColor3f(1,0,0);
		for (Vertex_iterator vi = data->mesh.p.vertices_begin(); vi!=data->mesh.p.vertices_end(); vi++,counter++) {
			char strBuf[10];
			sprintf(strBuf,"%d",counter);
			renderText(vi->point().x(),vi->point().y(),vi->point().z(), strBuf);
		}
		glColor3f(0,1,0);
		counter=0;
		for (Facet_iterator fi = data->mesh.p.facets_begin(); fi!=data->mesh.p.facets_end(); fi++,counter++) {
			char strBuf[10];
			sprintf(strBuf,"%d",counter);
			renderText(fi->center().x(),fi->center().y(),fi->center().z(), strBuf);
		}
	}
	
	if (debug_info_one_) { //show a specific vertex id and facet id
		if (debug_vertex_id < data->mesh.p.size_of_vertices()) {
			glColor3f(0.5,0,0);
			char strBuf[10];
			sprintf(strBuf,"%d",debug_vertex_id);
			Vertex_iterator vi = data->mesh.p.vertices_begin();
			for(int i=0;i<debug_vertex_id-1;i++) ++vi;
			render_point = vi->point() + scaling_factor/2*vi->normal();
			renderText(render_point.x(),render_point.y(),render_point.z(), strBuf);
			
			glColor3f(1,0,0);
			glBegin(GL_POINTS);
			render_point = vi->point() + scaling_factor/10*vi->normal();
			glVertex3f(render_point.x(),render_point.y(),render_point.z());
			glEnd();
		}
		
		if (debug_triangle_id < data->mesh.p.size_of_facets()) {
			glColor3f(0,0.5,0);
			char strBuf[10];
			sprintf(strBuf,"%d",debug_triangle_id);
			Facet_iterator fi = data->mesh.p.facets_begin();
			for(int j=0;j<debug_triangle_id-1;j++) ++fi;
			render_point = fi->center() + scaling_factor/2*fi->normal();
			renderText(render_point.x(),render_point.y(),render_point.z(), strBuf);
			
			glBegin(GL_TRIANGLES);
			HF_circulator hf= fi->facet_begin();
			
			
			glColor3f(0,1,0);
			Vector the_normal = fi->normal();
			do {
				glNormal3f(the_normal .x(),the_normal .y(),the_normal .z());
				Point coord =  hf->vertex()->point();// + scaling_factor/10*fi->normal();;
				glVertex3f(coord.x(),coord.y(),coord.z());
			} while (++hf != fi->facet_begin());
			glEnd();
			glBegin(GL_TRIANGLES);
			the_normal = the_normal*(-1);
			do {
				glNormal3f(the_normal .x(),the_normal .y(),the_normal .z());
				Point coord =  hf->vertex()->point();// + scaling_factor/10*fi->normal();;
				glVertex3f(coord.x(),coord.y(),coord.z());
			} while (--hf != fi->facet_begin());

			glEnd();

		
		}
	}
   	glDisable(GL_POLYGON_OFFSET_FILL);

//	glDepthRange (0.0, 0.9); /* Draw overlying geometry */
	if (curvature_) gl->DrawGeometryCurvature();
	if (deltas_) gl->DrawGeometryDeltas(); //delta_scale
	if (bounding_box_) gl->RenderBounds(data->mesh.getBoundingBox());
//	if (normals_) gl->DrawGeometryNormals(geometry_mode);
	if (normals_) gl->DrawGeometryColorDeriv(geometry_mode);	

	if (edges_) {

		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		//glColor3f(0.2,0.2,0.2);
		gl->DrawGeometry(GL_TRIANGLES,'N',0.2,0.2,0.2);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
/*		//glEnable(GL_POLYGON_OFFSET_LINE);
		//glPolygonOffset(-1.0f, -1.0f);
		glSetNiceLineWidth(1.0);
		//glLineWidth(2);
		glDisable(GL_LIGHTING);
		glColor3f(0.2,0.2,0.2);
		glBegin(GL_LINES);
		for ( Halfedge_iterator j = data->mesh.p.edges_begin(); j != data->mesh.p.edges_end(); ++j){
			Point tmp = j->vertex()->point();
			glVertex3f(tmp.x(),tmp.y(),tmp.z());
			tmp = j->opposite()->vertex()->point();
			glVertex3f(tmp.x(),tmp.y(),tmp.z());
	
		}
		glEnd();
		//glDisable(GL_POLYGON_OFFSET_LINE);		
		glLineWidth(1.0);*/
		glEnable(GL_LIGHTING);
	}
	
	
	
	if (segment_intersections_) {
		glLineWidth(10.0);
		glDisable(GL_LIGHTING);
		glBegin(GL_LINES);
		
		glColor3f(0.1,0.1,1);
		// intersection segments
		for ( std::vector<KernelExact::Segment_3*>::iterator j = data->mesh.inter_segments.begin(); j != data->mesh.inter_segments.end(); ++j){
			glVertex3f(TO_FLOAT((*j)->source().x()),TO_FLOAT((*j)->source().y()),TO_FLOAT((*j)->source().z()));
			glVertex3f(TO_FLOAT((*j)->target().x()),TO_FLOAT((*j)->target().y()),TO_FLOAT((*j)->target().z()));
		}
		glEnd();
		glLineWidth(1.0);
		glEnable(GL_LIGHTING);
	}
	
	if (motion_field_) {
		glDisable(GL_LIGHTING);
		for (Vertex_iterator vi = data->mesh.p.vertices_begin(); vi!=data->mesh.p.vertices_end(); vi++) {
			glBegin(GL_LINES);
			glVertex3f(TO_FLOAT(vi->point().x()),TO_FLOAT(vi->point().y()),TO_FLOAT(vi->point().z()));
			if (vi->vh_dist>0) 
				glColor3f(0, 1, 0);
			else
				glColor3f(0.3, 0.8, 1);
			
			render_point = vi->point() - vi->motion_vec;// * scalingFactor / data->mesh.vertices[i]->getMeanCurvatureFlow().Norm();
			glVertex3f(TO_FLOAT(render_point.x()),TO_FLOAT(render_point.y()),TO_FLOAT(render_point.z()));
			glEnd();
		}
		glEnable(GL_LIGHTING);
	}
	
	if (segment_delaunay_) {
		glLineWidth(7.0);		
		glDisable(GL_LIGHTING);
		glBegin(GL_LINES);
		glColor3f(0.5,0.5,1);

		//delaunay segments
		int local_counter=0;
		for (Facet_iterator fi = data->mesh.p.facets_begin(); fi!=data->mesh.p.facets_end(); fi++,local_counter++) {
			if ((fi->cdt!=NULL) && ((debug_info_one_==false) || (local_counter==debug_triangle_id-1)))
				for ( CDT::Finite_edges_iterator eit = fi->cdt->finite_edges_begin(); eit != fi->cdt->finite_edges_end(); ++eit )
				{
					if ( fi->cdt->is_constrained ( *eit ) ) 
						glColor3f(0.3,0.3,1);
					else
						glColor3f(0.5,0.5,1);
					
					CDT::Edge pp = *eit;
					CDT::Face_handle f = pp.first;
					int fedge = pp.second;
					
					KernelExact::Point_3 p1 = f->vertex ( fi->cdt->cw ( fedge ) )->point_3d;
					KernelExact::Point_3 p2 = f->vertex ( fi->cdt->ccw ( fedge ) )->point_3d;


					glVertex3d(CGAL::to_double(p1.x()),CGAL::to_double(p1.y()),CGAL::to_double(p1.z()));
					glVertex3d(CGAL::to_double(p2.x()),CGAL::to_double(p2.y()),CGAL::to_double(p2.z()));
				}
		}

		glEnd();
		glLineWidth(1.0);
		glEnable(GL_LIGHTING);
	}
	
	
	if (flatShading_)
		glShadeModel(GL_FLAT);
	else
		glShadeModel(GL_SMOOTH);
	
	
	if(lighting_) glEnable(GL_LIGHTING);
	else glDisable(GL_LIGHTING);
	data->mesh.unlock();
	
	// Restore the original (world) coordinate system
	glPopMatrix();	

	
}

/*********************************************************************************/
void MVViewer::initSceneRadius() {
//	setSceneCenter( qglviewer::Vec(0,0,0));
	float* bounding_box=data->mesh.getBoundingBox();
	setSceneCenter( qglviewer::Vec((bounding_box[0]+bounding_box[3])/2,(bounding_box[1]+bounding_box[4])/2,(bounding_box[2]+bounding_box[5])/2));	

	if (data==NULL) return;
	double rad=0;
	double n;
	if (cameras_) {
		for(int i=0;i<data->ncameras;i++){
			Vector3 tmpCenter = data->cameras[i].Center();
			n = ( sceneCenter()-qglviewer::Vec(tmpCenter(1),tmpCenter(2),tmpCenter(3) ) ).norm();
			assign_if_greater(rad,n);
		}
	}

	for(int i=0;i<6;i+=3) {
		n = (sceneCenter()-qglviewer::Vec(bounding_box[i],bounding_box[i+1],bounding_box[i+2] ) ).norm();
		assign_if_greater(rad,n);
	}
	cout << "scene radius" << rad << " scene center " <<(bounding_box[0]+bounding_box[3])/2<<","<<(bounding_box[1]+bounding_box[4])/2<<","<<(bounding_box[2]+bounding_box[5])/2 <<endl;
	cout << "and bounding box "<< bounding_box[0]<<","<<  bounding_box[1]<<","<<  bounding_box[2] <<endl;
	cout << bounding_box[3]<<","<<  bounding_box[4]<<","<<  bounding_box[5] <<endl;
	
	if (rad!=0) setSceneRadius( rad*0.8);
	else setSceneRadius(1);
	showEntireScene();
}

/*********************************************************************************/
void MVViewer::init() {
	
	// Add custom key description (see keyPressEvent).
	setKeyDescription(Qt::Key_W, "Toggles wire frame display");
	setKeyDescription(Qt::Key_F, "Toggles flat shading display");
	setKeyDescription(Qt::Key_R, "Remeshing");	
	setKeyDescription(Qt::Key_L, "Toggles ligthing");	
	setKeyDescription(Qt::Key_C, "Toggles cameras");
	setKeyDescription(Qt::Key_Z, "Background Color - invert");
	setKeyDescription(Qt::Key_T, "Rotate Geometry Mode");			
	setKeyDescription(Qt::Key_Q, "Rotate Quality Mode");			
	setKeyDescription(Qt::Key_N, "Toggles normals");			
	setKeyDescription(Qt::Key_Plus, "Increments the current camera");		
	setKeyDescription(Qt::Key_Minus, "Decrements the current camera");			
	/*		setSceneCenter( qglviewer::Vec( (ls.nb.minx()+ls.nb.maxx())/2.,
	(ls.nb.miny()+ls.nb.maxy())/2.,
	(ls.nb.minz()+ls.nb.maxz())/2.) );
*/
	glc->makeCurrent();
//	setHandlerKeyboardModifiers(QGLViewer::CAMERA, Qt::NoModifier);
//	setHandlerKeyboardModifiers(QGLViewer::FRAME,  Qt::ControlModifier);
	setHandlerKeyboardModifiers(QGLViewer::FRAME, Qt::AltModifier);
//	setMouseBinding(Qt::ShiftModifier | Qt::RightButton, QGLViewer::FRAME, QGLViewer::TRANSLATE, false);	
	setMouseBinding(Qt::ALT + Qt::SHIFT + Qt::LeftButton, FRAME, QGLViewer::TRANSLATE);
	
//	setMouseBinding(Qt::LeftButton, FRAME, DRIVE);
#ifdef GL_RESCALE_NORMAL  // OpenGL 1.2 Only...
	glEnable(GL_RESCALE_NORMAL);
#endif
	
	initSceneRadius();
	setManipulatedFrame(new qglviewer::ManipulatedFrame());	
	gl->InitInteractiveMode();
	glc->doneCurrent();
}



/*********************************************************************************/
void MVViewer::keyPressEvent(QKeyEvent *e) {
	if (data==NULL) return;
	
	// Get event modifiers key
#if QT_VERSION < 0x040000
	// Bug in Qt : use 0x0f00 instead of Qt::KeyButtonMask with Qt versions < 3.1
	const Qt::ButtonState modifiers = (Qt::ButtonState)(e->state() & Qt::KeyButtonMask);
#else
	const Qt::KeyboardModifiers modifiers = e->modifiers();
#endif
	
	if ((e->key()==Qt::Key_W) && (modifiers==Qt::NoButton)) {
		if (polygon_draw_mode == 'F') { setPolygonDrawingMode('P');}
		else if (polygon_draw_mode == 'P') { setPolygonDrawingMode('W');}
		else if (polygon_draw_mode == 'W') { setPolygonDrawingMode('F');}
    }
	else if ((e->key()==Qt::Key_F) && (modifiers==Qt::NoButton)) {
		flatShading_ = !flatShading_;
	}
	else if(e->key()==Qt::Key_R) {
		int old_no_faces = data->mesh.p.size_of_facets() ;
		cout << "REMESHING" << endl;
		data->mesh.remesh();
 		cout << " - no. of facets increased from " << old_no_faces << " to " << data->mesh.p.size_of_facets() << endl;
	}
	else if(e->key()==Qt::Key_L) {
		setLighting(!lighting_);
	}
	else if(e->key()==Qt::Key_C) {
		setCameras(!cameras_);
	}
	else if(e->key()==Qt::Key_Z) {
		for(int i=0;i<3;i++) bg_color[i]=1.0-bg_color[i];
	}
	else if(e->key()==Qt::Key_E) {
		printf("Expanding\n");		
		data->mesh.dilate(scaling_factor);
		
	}	
	else if(e->key()==Qt::Key_S) {
		printf("Smooth\n");		
		data->mesh.smooth(1);
		
	}	
	else if(e->key()==Qt::Key_I) {
		printf("Init / Re-Load Mesh\n");		
		data->mesh.reload();
	}	
	else if(e->key()==Qt::Key_N) {
		setNormals(!normals_);
	}	
	else if(e->key()==Qt::Key_M) {
		setMeanCurvature(!curvature_);
	}	
	
	else if(e->key()==Qt::Key_T) {
		if (geometry_mode == 'T') { setGeometryMode('N');} //texture
		else if (geometry_mode == 'N') { setGeometryMode('Q');}	 //normal (white)
		else if (geometry_mode == 'Q') { setGeometryMode('q');} // quality
		else if (geometry_mode == 'q') { setGeometryMode('W');} // quality difference
		else if (geometry_mode == 'W') { setGeometryMode('C');}	// weights
		else if (geometry_mode == 'C') { setGeometryMode('R');} // color
		else if (geometry_mode == 'R') { setGeometryMode('T');}	// facets
	}	
	
	else if(e->key()==Qt::Key_Q) {
		if (quality_mode == Mesh::Qual_Color) { setQualityMode(Mesh::Qual_Color_Deriv); }
		else if (quality_mode == Mesh::Qual_Color_Deriv) { setQualityMode(Mesh::Qual_Mean_Curv); }
		else if (quality_mode == Mesh::Qual_Mean_Curv) { setQualityMode(Mesh::Qual_Mean_Curv_Deriv); }
		else if (quality_mode == Mesh::Qual_Mean_Curv_Deriv) { setQualityMode(Mesh::Qual_Gaussian_Curv); }
		else if (quality_mode == Mesh::Qual_Gaussian_Curv) { setQualityMode(Mesh::Qual_Gaussian_Curv_Deriv); }
		else if (quality_mode == Mesh::Qual_Gaussian_Curv_Deriv) { setQualityMode(Mesh::Qual_Color); }
	}	

	
	else if(e->key()==Qt::Key_B) {
		bounding_box_ = !bounding_box_;
	}	
	else if(e->key()==Qt::Key_P) {
		if (geom_enum == GL_TRIANGLES)		
			geom_enum = GL_POINTS;
		else
			geom_enum = GL_TRIANGLES;
	}	
	else if( (e->key()==Qt::Key_Plus) || (e->key()==Qt::Key_Equal)) {
		gl->currentCamera = (gl->currentCamera + 1) % data->ncameras;
	}
	else if(e->key()==Qt::Key_Minus) {
		gl->currentCamera = (gl->currentCamera - 1 + data->ncameras) % data->ncameras;
	}
	else QGLViewer::keyPressEvent(e);
	
	updateGL();
}

/*********************************************************************************/
void MVViewer::animate() {
	//		ls.update_beta();
	//		ls.evolve();
	
	if(counter>0) {
		//~ ls.update_depthmaps();
		//~ ls.update_colors();
		//~ ls.update_Sigma();
		//~ ls.update_beta();
		counter=0;
	}
	counter++;
	
	updateGL();
}



/////////////////////////////////////////////////////////////////////////////////////
void MVViewer::setMeanCurvature(bool state) {
	printf("Toggling Mean Curvature (now=%d)\n",state);		
	curvature_ = state;
	updateGL();
}

/////////////////////////////////////////////////////////////////////////////////////
void MVViewer::setEdges(bool state) {
	printf("Toggling Edges(now=%d)\n",state);		
	edges_ = state;
	updateGL();
}

/////////////////////////////////////////////////////////////////////////////////////
void MVViewer::setNormals(bool state) {
	printf("Toggling Normals (now=%d)\n",state);		
	normals_ = state;
	updateGL();
}

/////////////////////////////////////////////////////////////////////////////////////
void MVViewer::setDeltas(bool state, double value) {
	printf("Toggling Distance from Visual Hull (now=%d)\n",state);		
	deltas_ = state;
	delta_scale = value;
	updateGL();
}


/////////////////////////////////////////////////////////////////////////////////////
void MVViewer::setTextures(bool state) {
	printf("Toggling Textures (now=%d)\n",state);		
	textures_ = state;

	glc->makeCurrent();		
	if (textures_) {
		for(int i=0;i<data->ncameras;i++) {
			if (textures_) gl->InitTexture(i,data->images[i]);
		}
	}
	else
		gl->DeleteTexture();
	glc->doneCurrent();		
	updateGL();
}


void MVViewer::setPolygonDrawingMode(char mode) {
	cout << "Polygon Drawing Mode =" << mode << endl;
	polygon_draw_mode=mode;
	updateGL();
}

/////////////////////////////////////////////////////////////////////////////////////
void MVViewer::setCameras(bool state) {
	printf("Toggling Cameras (now=%d)\n",state);		
	cameras_ = state;
	initSceneRadius();	
	updateGL();
}


/////////////////////////////////////////////////////////////////////////////////////
void MVViewer::setMeshTextureOne(bool state) {
	setGeometryMode('T');
}

/////////////////////////////////////////////////////////////////////////////////////
void MVViewer::setMeshDistinctTriangles(bool state) {
	setGeometryMode('D');
}

/////////////////////////////////////////////////////////////////////////////////////
void MVViewer::setMeshWhite(bool state) {
	setGeometryMode('N');
}

/////////////////////////////////////////////////////////////////////////////////////
void MVViewer::setMeshWeights(bool state) {
	setGeometryMode('W');
}

/////////////////////////////////////////////////////////////////////////////////////
void MVViewer::setSmoothShading(bool state) {
	gl->smoothShading=state;
	updateGL();
}


void MVViewer::setQualityMode(Mesh::QualityComputationMethod q_mode) {
	quality_mode = q_mode;
	data->mesh.setVertexQuality(quality_mode);
	setGeometryMode('Q');
}

/////////////////////////////////////////////////////////////////////////////////////
void MVViewer::setGeometryMode(char mode) {
	geometry_mode = mode;
	cout << "Geometry mode: " << mode << " - ";
	if (geometry_mode == 'N') { cout << "Normal";}	
	else if (geometry_mode == 'Q') { 
		cout << "Quality = ";
		if (quality_mode == Mesh::Qual_Color) { cout << "Color";}
		else if (quality_mode == Mesh::Qual_Color_Deriv) { cout << "Color Deriv";}
		else if (quality_mode == Mesh::Qual_Mean_Curv) { cout << "Mean Curv";}		
		else if (quality_mode == Mesh::Qual_Mean_Curv_Deriv) { cout << "Mean Curv Deriv";}		
		else if (quality_mode == Mesh::Qual_Gaussian_Curv) { cout << "Gaussian Curv";}		
		else if (quality_mode == Mesh::Qual_Gaussian_Curv_Deriv) { cout << "Gaussian Curv Deriv";}		
	}
	else if (geometry_mode == 'q') { cout << "Quality Derivative";}
	else if (geometry_mode == 'W') { cout << "Weights";}
	else if (geometry_mode == 'C') { cout << "Colors";}
	else if (geometry_mode == 'R') { cout << "TransforMesh Facet Colors";}
	else if (geometry_mode == 'T') { cout << "Texture";}

	cout << endl;
	if (geometry_mode == 'T')
		texturing_ = true;
	else
		texturing_ = false;

	updateGL();
}

/////////////////////////////////////////////////////////////////////////////////////
void MVViewer::setFaceCullingMode(char mode) {
	glc->makeCurrent();	
	switch (mode) {
		case 'N':		
			glDisable(GL_CULL_FACE);
			break;
		case 'F':
			glEnable(GL_CULL_FACE);
			glCullFace(GL_FRONT);
			break;
		case 'B':
			glEnable(GL_CULL_FACE);
			glCullFace(GL_BACK);
			break;			
	}
	glc->doneCurrent();		
	updateGL();	
}


void MVViewer::glSetNiceLineWidth(double size) {
	glEnable (GL_LINE_SMOOTH);
	glEnable (GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glHint (GL_LINE_SMOOTH_HINT, GL_NICEST);
	glLineWidth (size);
}
	


/////////////////////////////////////////////////////////////////////////////////////
void MVViewer::setSegmentIntersections(bool state) {
	printf("Toggling Segment Intersections(now=%d)\n",state);		
	segment_intersections_ = state;
	updateGL();
}

/////////////////////////////////////////////////////////////////////////////////////
void MVViewer::setSegmentDelaunay(bool state) {
	printf("Toggling Segment Delaunay(now=%d)\n",state);		
	segment_delaunay_= state;
	updateGL();
}

/////////////////////////////////////////////////////////////////////////////////////
void MVViewer::setDebugInfo(bool state) {
	printf("Toggling Visual Debug Info(now=%d)\n",state);		
	debug_info_= state;
	updateGL();
}

/////////////////////////////////////////////////////////////////////////////////////
void MVViewer::setMotionField(bool state) {
	printf("Toggling View Motion Field (now=%d)\n",state);		
	motion_field_= state;
	updateGL();
}

/////////////////////////////////////////////////////////////////////////////////////
void MVViewer::setLighting(bool state) {
	printf("Toggling Lighting Mode(now=%d)\n",state);		
	lighting_ = state;
	updateGL();
}

/////////////////////////////////////////////////////////////////////////////////////
void MVViewer::setDebugInfoOne(bool state, int theVertexId, int theTriangleId) {
	if (state!=debug_info_one_)
		printf("Toggling Show ID (now=%d) of vertex=%d and triangle=%d\n",state, theVertexId, theTriangleId);
	debug_vertex_id = theVertexId;
	debug_triangle_id = theTriangleId;
	debug_info_one_= state;
	updateGL();
}

/////////////////////////////////////////////////////////////////////////////////////
void MVViewer::saveSnap(QString filename) {
	glc->makeCurrent();
	saveSnapshot(filename,true);
	glc->doneCurrent();	
}