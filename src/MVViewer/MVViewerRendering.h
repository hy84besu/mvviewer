#ifndef MVVIEWERRENDERING_H
#define MVVIEWERRENDERING_H

#include <cassert>
#include <iostream>
#include <vector>
#include <cmath>
#include <cfloat>
#include <string.h>
#include "CImg_mod.h"
#include "Camera.h"
#include "OpenGLContext.h"
#include "MVData.h"
#include "ColorMap.h"
using namespace cimg_library;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Classe pour le rendu OpenGL
// Cette classe regoupe des fonctions pour la stereovision, l'estimation du mouvement, et le rendu avec ou sans texture
class MVViewerRendering {
protected:
	OpenGLContext *glc;
	MVData *data;

	int width,height;                 // Dimensions du buffer OpenGL
	GLsizei ncamera;                  // Nombre de cameras
	GLuint surface_list, camera_list; // Listes pour le dessin du maillage et pour les matrices de camera
	GLuint *texture_ids;              // Textures pour les images vues depuis les differentes cameras
	GLuint *depth_ids;                // Textures pour les z-buffer dans chaque camera, utilisees pour le shadow mapping
	int texture_width,texture_height; // Taille des textures (puissances de deux immediatement superieures aux dimensions du framebuffer)
	Vector3 *center;                  // Centres des cameras
	float *zNear, *zFar;              // Clipping planes des cameras
	int *camWidth, *camHeight;
public:
	int currentCamera;
	bool colorTriangles;
	bool smoothShading;
	double sc;	//normals scaling factor

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Constructeur / destructeur
	MVViewerRendering(OpenGLContext* _glc, MVData* _data, int w, int h) : glc(_glc), data(_data) {

		width = w;
		height = h;
		ncamera = data->ncameras;
		sc = 0.003; 

		currentCamera = 1;
		colorTriangles = false;
		smoothShading = true;

		glc->makeCurrent();
		// Les dimensions de la texture doivent etre des puissances de 2
		// On prend les puissances de 2 immediatement superieures
		// Doit pouvoir etre vire en utilisant une extension des cartes graphiques recentes
		texture_width = 1 << int(ceil(log(float(width)) / log(2.0)));
		texture_height = 1 << int(ceil(log(float(height)) / log(2.0)));

		texture_width = width;
		texture_height = height;
		

		glTexEnvf(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_REPLACE);

		// Display list pour la surface
		surface_list = glGenLists(1);
		//		printf("surface list %d\n",surface_list);

		// Display list pour les vues (cameras + une vue auxiliaire)
		camera_list = glGenLists(ncamera+1);

		//		printf("camera list %d\n",camera_list);

		// Identifiant pour les textures
		texture_ids = new GLuint[ncamera];
		depth_ids = new GLuint[ncamera];
		glGenTextures(ncamera,texture_ids);
		glGenTextures(ncamera,depth_ids);

		// Initialisation des texture z-buffer
		float *zero = new float[texture_width*texture_height];
		memset(zero,0,texture_width*texture_height*sizeof(float));
		const float border[4] = { 0,0,0,1};
		for (int i=0;i<ncamera;i++) {
			glBindTexture(GL_TEXTURE_2D,depth_ids[i]);
			glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_BORDER);
			glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP_TO_BORDER);

			glTexParameterfv(GL_TEXTURE_2D,GL_TEXTURE_BORDER_COLOR,border);
			glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_COMPARE_FUNC,GL_LEQUAL);
			glTexParameteri(GL_TEXTURE_2D,GL_DEPTH_TEXTURE_MODE,GL_LUMINANCE);
			glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_COMPARE_MODE,GL_COMPARE_R_TO_TEXTURE);
			glTexImage2D(GL_TEXTURE_2D,0,GL_DEPTH_COMPONENT,texture_width,texture_height,0,GL_DEPTH_COMPONENT,GL_FLOAT,zero);
		}
		delete[] zero;
		glc->doneCurrent();
		// Centre et clipping planes des cameras
		center = new Vector3[ncamera+1];
		zFar = new float[ncamera+1];
		zNear = new float[ncamera+1];
		camWidth = new int[ncamera+1];
		camHeight = new int[ncamera+1];

	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	~MVViewerRendering() {
		glc->makeCurrent();

		// Liberation des listes OpenGL
		glDeleteLists(surface_list,1);
		glDeleteLists(camera_list,ncamera+1);

		// Destruction des textures
		glDeleteTextures(ncamera,texture_ids);
		glDeleteTextures(ncamera,depth_ids);
		glc->doneCurrent();
		
		delete [] texture_ids;
		delete [] depth_ids;

		// Menage
		delete [] center;
		delete [] zNear;
		delete [] zFar;

		delete [] camWidth;
		delete [] camHeight;		
		
		//		pbuffer.Deactivate();

	}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void InitInteractiveMode() { // gotto brush up this lightning a bit
		glc->makeCurrent();
/*		double ratio=25;
		GLfloat pos1[] = { ratio* 0.1,  ratio*0.1, ratio*-0.02, 0.0};
		GLfloat pos2[] = { ratio*-0.1,  ratio*0.1, ratio*-0.02, 0.0};
		GLfloat pos3[] = { ratio* 0.0,  ratio*0.0, ratio* 0.1,  0.0};
		GLfloat col1[] = { 0.7,  0.7,  0.8,  1.0};
		GLfloat col2[] = { 0.8,  0.7,  0.7,  1.0};
		GLfloat col3[] = { 1.0,  1.0,  1.0,  1.0};
		
		glEnable(GL_LIGHT0);    
		glLightfv(GL_LIGHT0,GL_POSITION, pos1);
		glLightfv(GL_LIGHT0,GL_DIFFUSE,  col1);
		glLightfv(GL_LIGHT0,GL_SPECULAR, col1);
		
		glEnable(GL_LIGHT1);  
		glLightfv(GL_LIGHT1,GL_POSITION, pos2);
		glLightfv(GL_LIGHT1,GL_DIFFUSE,  col2);
		glLightfv(GL_LIGHT1,GL_SPECULAR, col2);
		
		glEnable(GL_LIGHT2);  
		glLightfv(GL_LIGHT2,GL_POSITION, pos3);
		glLightfv(GL_LIGHT2,GL_DIFFUSE,  col3);
		glLightfv(GL_LIGHT2,GL_SPECULAR, col3);
*/
		GLfloat ambient[] = {0.1f,0.1f,0.1f,1.0f};
		glLightModelfv(GL_LIGHT_MODEL_AMBIENT,ambient);
		glc->doneCurrent();
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void DrawGeometry(GLenum mode=GL_TRIANGLES,char colorMode='N', float col_r=0.85, float col_g=0.80, float col_b=0.80) {
		float lower_qual, upper_qual;	
		if ( (colorMode=='Q'))
			data->mesh.computeQualityPercentileBoundaries(lower_qual,upper_qual,0.01);
		if ( (colorMode=='q'))
			data->mesh.computeQualityDiffPercentileBoundaries(lower_qual,upper_qual,0.01);

//		glColor3f(1,1,1);
		glBegin(GL_TRIANGLES);
		int local_counter=0;
		for (Facet_iterator fi = data->mesh.p.facets_begin(); fi != data->mesh.p.facets_end(); ++fi) {
			local_counter++;
			HF_circulator hf =fi->facet_begin();
			do {
				Vector local_normal;
				Vertex_handle local_v;
				if (smoothShading==false) {
					local_v = fi->halfedge()->vertex();
					local_normal = fi->normal();
				}	
				else {
					local_v = hf->vertex();
					local_normal= hf->vertex()->normal();
				}
				Point p = hf->vertex()->point();
				float cols[3];
				if ( (colorMode=='Q') ) { //quality - set separately
					float qual = local_v->quality();
//					if (qual>0) qual = std::min(qual/upper_qual,1.0f);
//					else qual= std::max(-1.0f,qual/std::abs(lower_qual));
//					qual = qual/2+0.5;
					qual = (qual -lower_qual)/ (upper_qual - lower_qual);
					qual = std::max(std::min(1.0f,qual),0.0f);
					ColorMap::jet(qual,cols);

				}
				if ( (colorMode=='q') ) { //quality difference - set separately
					float qual = local_v->quality()-local_v->quality_prev();
					//					if (qual>0) qual = std::min(qual/upper_qual,1.0f);
					//					else qual= std::max(-1.0f,qual/std::abs(lower_qual));
					//					qual = qual/2+0.5;
					qual = (qual -lower_qual)/ (upper_qual - lower_qual);
					qual = std::max(std::min(1.0f,qual),0.0f);
					ColorMap::jet(qual,cols);
					
				}
				if (colorMode=='W') { // weights
					cols[0] = 1.0;
					cols[2] = cols[1] = 1.0 -local_v->weight;
				}
				if (colorMode=='C') { // vertex color
					for(int xxx=0;xxx<3;xxx++)
						cols[xxx] = local_v->color[xxx];
				}
				if (colorMode=='R') { // TransforMesh facet colors
					for(int xxx=0;xxx<3;xxx++)
						cols[xxx] = 0.3f;

					if ((data->mesh.interactive_colors_max_facet_no<0) || (local_counter<data->mesh.interactive_colors_max_facet_no)) {
						if (fi->removal_status=='S') cols[0]=1.0f;
						else if (fi->removal_status=='V') cols[1]=1.0f;
						else if (fi->removal_status=='P') cols[2]=1.0f;
						else
							for(int xxx=0;xxx<3;xxx++)
								cols[xxx] = 0.85f;
					}
					else {
						for(int xxx=0;xxx<3;xxx++)
							cols[xxx] = 0.7f;
					}
						
				}

				if (colorMode=='N') { // default colors
					cols[0] = col_r; cols[1] = col_g; cols[2] = col_b;
				}
					
				glColor3f(cols[0], cols[1], cols[2]);
				glNormal3f(local_normal.x(),local_normal.y(),local_normal.z());
				glTexCoord3f(p.x(),p.y(), p.z());
				glVertex3f(p.x(),p.y(), p.z());


			} while (++hf != fi->facet_begin());
		}
		glEnd();
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void DrawGeometryNormals(char colorMode='N') {
		data->mesh.lock();
		glDisable(GL_LIGHTING);
		glColor3f(1,0,0);
		glBegin(GL_LINES);
		for (Vertex_iterator vi = data->mesh.p.vertices_begin(); vi != data->mesh.p.vertices_end(); ++vi) {
				Vector tmp = vi->normal();
				Point p = vi->point();
				glVertex3f(p.x(),p.y(), p.z());
				p = p + vi->normal()*sc;
				glVertex3f(p.x(),p.y(), p.z());
		}
		glEnd();
		glEnable(GL_LIGHTING);
		data->mesh.unlock();
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void DrawGeometryColorDeriv(char colorMode='N') {
		data->mesh.lock();
		glDisable(GL_LIGHTING);
		glColor3f(1,0,0);
		glBegin(GL_LINES);
		for (Vertex_iterator vi = data->mesh.p.vertices_begin(); vi != data->mesh.p.vertices_end(); ++vi) {
			Vector tmp = vi->normal();
			Point p = vi->point();
			glVertex3f(p.x(),p.y(), p.z());
			p = p + vi->qual_vect*sc;
			glVertex3f(p.x(),p.y(), p.z());
		}
		glEnd();
		glEnable(GL_LIGHTING);
		data->mesh.unlock();
	}
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void DrawGeometryDeltas() {
		Point tmpPoint;
		glDisable(GL_LIGHTING);
		for (Vertex_iterator vi = data->mesh.p.vertices_begin(); vi!=data->mesh.p.vertices_end(); vi++) {
			glBegin(GL_LINES);
			glVertex3f(TO_FLOAT(vi->point().x()),TO_FLOAT(vi->point().y()),TO_FLOAT(vi->point().z()));
			if (vi->vh_dist>0) 
				glColor3f(0, 1, 0);
			else
				glColor3f(0, 0, 1);

			tmpPoint = vi->point() + vi->normal()*vi->vh_dist;// * scalingFactor / data->mesh.vertices[i]->getMeanCurvatureFlow().Norm();
			glVertex3f(TO_FLOAT(tmpPoint.x()),TO_FLOAT(tmpPoint.y()),TO_FLOAT(tmpPoint.z()));
			glEnd();
		}
		glEnable(GL_LIGHTING);
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void DrawGeometryCurvature() {
		Point tmpPoint;
		Vector tmpVector;
		glDisable(GL_LIGHTING);
		glBegin(GL_POINTS);
		glColor3f(0.5,0.5,0.5);
		for (Vertex_iterator vi = data->mesh.p.vertices_begin(); vi!=data->mesh.p.vertices_end(); vi++)
			glVertex3f(TO_FLOAT(vi->point().x()),TO_FLOAT(vi->point().y()),TO_FLOAT(vi->point().z()));
		glEnd();


		for (Vertex_iterator vi = data->mesh.p.vertices_begin(); vi!=data->mesh.p.vertices_end(); vi++) {
			glBegin(GL_LINES);
			glColor3f(0, 0, 1);			//*data->mesh.vertices[i]->getMeanCurvatureFlow().Norm()
			glVertex3f(TO_FLOAT(vi->point().x()),TO_FLOAT(vi->point().y()),TO_FLOAT(vi->point().z()));
			//tmpNormal = *data->mesh.vertices[i] + data->mesh.vertices[i]->getMeanCurvatureFlow();// * scalingFactor / data->mesh.vertices[i]->getMeanCurvatureFlow().Norm();
			tmpVector = (vi->point()-Point(0,0,0)) + vi->laplacian()*vi->weight;// * scalingFactor / data->mesh.vertices[i]->getMeanCurvatureFlow().Norm();
			glVertex3f(TO_FLOAT(tmpVector.x()),TO_FLOAT(tmpVector.y()),TO_FLOAT(tmpVector.z()));
			glEnd();
		}
		glEnable(GL_LIGHTING);

	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void DrawGeometryWithTextures() {

		int itexture = currentCamera;

		// Matrice de texture
		glMatrixMode(GL_TEXTURE);
		glLoadIdentity();
		float scalex = float(width) / float(texture_width) / 2;
		float scaley = float(height) / float(texture_height) / 2;
		scalex=0.5;scaley=0.5;
		glTranslatef(scalex,scaley,0.5);
		glScalef(scalex,scaley,0.5);
		glCallList(camera_list+itexture);

		// draw the mask
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D,texture_ids[itexture]);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_BORDER);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP_TO_BORDER);
		
		DrawGeometry();
		glDisable(GL_TEXTURE_2D);

		glMatrixMode(GL_TEXTURE);
		glLoadIdentity();

	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void DrawCameras(double z) {
		//draw the camera centers
		glPointSize(3);
		glColor3f(0.6,1.,0.6);
		glDisable(GL_LIGHTING);
		glBegin(GL_POINTS);
		for(int i=0;i<data->ncameras;i++) {
			if (currentCamera == i)
				glColor3f(1,0,0.6);
//				glColor3f(0.2,0.8,0.2);			
			else
				glColor3f(0.2,1.,0.2);
			glVertex3f(center[i](1),center[i](2),center[i](3));
		}
		glEnd();


		for( int i=0; i<data->ncameras; i++ ) {
			glEnable(GL_LIGHTING);

			glPushMatrix();
			GLdouble tmp[4*4];
			data->cameras[i].Rigide2GL(tmp);
			glMultMatrixd(tmp);

			glColor4f( 1,1,1,1 );

			glEnable(GL_TEXTURE_2D);			// Enable Texture Mapping
			glBindTexture(GL_TEXTURE_2D, texture_ids[i]);		// Select Our Texture
			glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_BORDER);
			glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP_TO_BORDER);

			Vector3 dir[4];
			glNormal3f(0,0,-1);
			glBegin(GL_QUADS);

			dir[0] = data->cameras[i].IntrinsicPixelDirection(0,0);
			dir[0] = dir[0]* (z/dir[0](3));
			dir[1] = data->cameras[i].IntrinsicPixelDirection(0,data->imageHeight(i));
			dir[1] = dir[1]* (z/dir[1](3));
			dir[2] = data->cameras[i].IntrinsicPixelDirection(data->imageWidth(i),data->imageHeight(i));
			dir[2] = dir[2]* (z/dir[2](3));
			dir[3] = data->cameras[i].IntrinsicPixelDirection(data->imageWidth(i),0);
			dir[3] = dir[3]* (z/dir[3](3));

			glTexCoord2f(0.0f,0.0f);
			glVertex3dv(dir[0].coord);

			glTexCoord2f(1.0,0.0f);
			glVertex3dv(dir[3].coord);

			glTexCoord2f(1.0,1.0);
			glVertex3dv(dir[2].coord);

			glTexCoord2f(0.0f,1.0);
			glVertex3dv(dir[1].coord);


			glEnd();
			glDisable(GL_TEXTURE_2D);

			glDisable(GL_LIGHTING);
			glEnable(GL_LINE_SMOOTH_HINT);
			//			glLineWidth(0.1);
			glBegin(GL_LINES);
			if (currentCamera == i)
				glColor3f(1,0,0.6);
				//glColor3f(0.2,0.8,0.2);			
			else
				glColor3f(0.2,1.,0.2);
			
			for(int i=0;i<4;i++) {
				glVertex3f(0,0,0);
				glVertex3dv(dir[i].coord);
				glVertex3dv(dir[i].coord);
				glVertex3dv(dir[(i+1)%4].coord);

			}
			glEnd();

			glPopMatrix();
		}
		
		//draw the text
		for(int i=0;i<data->ncameras;i++) {
			glColor3f(1,0,0);
			char strBuf[10];
			sprintf(strBuf,"%d",i);
			glc->viewer->renderText(center[i](1),center[i](2),center[i](3), strBuf);
		}
		glPointSize(1);
		glEnable(GL_LIGHTING);
	}

	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Initialisation d'une camera
	// On convertit notre objet Camera en liste OpenGL et on la memorise
	void InitCamera(int i, const Camera &camera, const float bounds[6] = 0) {

		glc->makeCurrent();

		// On verifie que la matrice de camera soit normalisee

//		assert(fabs(camera.matrix.GetRow(3).Norm()-1)<1e-3f);

		// On memorise le centre de la camera
		center[i] = camera.Center();
		camWidth[i] = camera.width;
		camHeight[i] = camera.height;		

		// Calcul des clipping planes optimaux des cameras
		zNear[i] = FLT_MAX;
		zFar[i] = - FLT_MAX;
		if (bounds) {
			for (int j=0;j<=1;j++)
				for (int k=0;k<=1;k++)
					for (int l=0;l<=1;l++) {
						const float x = bounds[3*j];
						const float y = bounds[3*k+1];
						const float z = bounds[3*l+2];
						float X,Y,Z;
						camera.rigide.FastMultiply(x,y,z,X,Y,Z);
						if (abs(Z)>zFar[i])
							zFar[i] = abs(Z);
						if (abs(Z)<zNear[i])
							zNear[i] = abs(Z);
					}

			// On verifie que l'objet ne soit pas derriere la camera
			assert(zNear[i]>0 && zFar[i]>0);
		}

		// Conversion de la camera en instructions OpenGL
		glNewList(camera_list+i, GL_COMPILE);

		glTranslatef(1/float(camera.width),1/float(camera.height),0); // Attention, en OpenGL, le centre du premier pixel est en (0.5,0.5) !!!!
		glFrustum(0, -float(camera.width)*zNear[i], 0, -float(camera.height)*zNear[i], zNear[i], zFar[i]);
		const GLfloat mat[16] = {
		                            -camera.matrix(1,1),-camera.matrix(2,1),-camera.matrix(3,1),0,
		                            -camera.matrix(1,2),-camera.matrix(2,2),-camera.matrix(3,2),0,
		                            -camera.matrix(1,3),-camera.matrix(2,3),-camera.matrix(3,3),0,
		                            -camera.vector(1),-camera.vector(2),-camera.vector(3),camera.matrix.GetRow(3).Norm()
		                        };
		glMultMatrixf(mat);
		glEndList();
		glc->doneCurrent();
	}


	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Initialisation d'une texture
	// Pour charger une image dans la carte graphique
	void InitTexture(int i, const CImg<> &img) {
		glc->makeCurrent();

		// On genere une texture de type unsigned byte et de dimensions egales a une puissance de 2
//		CImg<GLubyte> texture(img.dim,texture_width,texture_height);
		CImg<GLubyte> texture(img.dim,img.dimx(),img.dimy());		
		cimg_forXYZ(texture,k,x,y) texture(k,x,y) = GLubyte(img.pix2d(x,y,0,k));

		// On charge la texture dans la carte graphique
		glPixelStorei(GL_UNPACK_ALIGNMENT,1);
		glBindTexture(GL_TEXTURE_2D,texture_ids[i]);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP_TO_EDGE);

		//		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
		//		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );

		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
		if (img.dim == 1)
			glTexImage2D(GL_TEXTURE_2D,0,GL_LUMINANCE,img.dimx(),img.dimy(),0,GL_LUMINANCE,GL_UNSIGNED_BYTE,texture.data);
		else if (img.dim == 3)
			glTexImage2D(GL_TEXTURE_2D,0,GL_RGB,img.dimx(),img.dimy(),0,GL_RGB,GL_UNSIGNED_BYTE,texture.data);
		else
			assert(false);
		glc->doneCurrent();
	}
		
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////		
	void DeleteTexture() {
		glc->makeCurrent();
		glDeleteTextures(ncamera,texture_ids);
		glDeleteTextures(ncamera,depth_ids);
		glc->doneCurrent();
		
	}


	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Rendu du cube delimitant le domaine (pour la visu uniquement)
	void RenderBounds(const float bounds[6]) { 

		//        glClear(GL_COLOR_BUFFER_BIT);
		//        glDisable(GL_LIGHTING | GL_TEXTURE_2D);
		glBegin(GL_LINE_LOOP);
		glColor3f(0,0,0);
		glVertex3f(bounds[0],bounds[1],bounds[2]);
		glVertex3f(bounds[3],bounds[1],bounds[2]);
		glVertex3f(bounds[3],bounds[4],bounds[2]);
		glVertex3f(bounds[0],bounds[4],bounds[2]);
		glEnd();

		glBegin(GL_LINE_LOOP);
		glColor3f(0,0,0);
		glVertex3f(bounds[0],bounds[1],bounds[5]);
		glVertex3f(bounds[3],bounds[1],bounds[5]);
		glVertex3f(bounds[3],bounds[4],bounds[5]);
		glVertex3f(bounds[0],bounds[4],bounds[5]);
		glEnd();

		glBegin(GL_LINES);
		glVertex3f(bounds[0],bounds[1],bounds[2]);
		glVertex3f(bounds[0],bounds[1],bounds[5]);
		glVertex3f(bounds[3],bounds[1],bounds[2]);
		glVertex3f(bounds[3],bounds[1],bounds[5]);
		glVertex3f(bounds[3],bounds[4],bounds[2]);
		glVertex3f(bounds[3],bounds[4],bounds[5]);
		glVertex3f(bounds[0],bounds[4],bounds[2]);
		glVertex3f(bounds[0],bounds[4],bounds[5]);
		glEnd();

		// Recopie des couleurs en memoire
		//        glPixelStorei(GL_PACK_ALIGNMENT,4);
		//        glReadPixels(0,0,img.width,img.height,GL_RED,GL_FLOAT,img.data);
		//        img *= 255;
	}
};

#endif
