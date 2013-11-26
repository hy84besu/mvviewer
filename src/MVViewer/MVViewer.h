#ifndef _VIEWER_H_
#define _VIEWER_H_
#include <QGLViewer/qglviewer.h>
#include <QKeyEvent>

#include "MVData.h"
#include "Rigide.h"

using namespace std;
#include "MVViewerRendering.h"
#include "OpenGLContext.h"
#include "Resample.h"

class MVViewer : public QGLViewer
{
Q_OBJECT
public:
	MVViewer(QWidget *parent=0);
	void setMVData(MVData*);
	~MVViewer();

	MVData* data;
	int transformesh_colors_max_facet_no;
	
	static void glSetNiceLineWidth(double size);
	
	virtual void updateGL();
	virtual void makeCurrent() {glc->makeCurrent(); cout << "make current" << endl;}
	virtual void doneCurrent() {glc->doneCurrent();}	
	
	virtual void paintGL();
	virtual void resizeGL (int width, int height);
	virtual void initializeGL ();
	void saveSnap(QString filename);
public slots:
	
	void setScalingFactor(double newScale);
	void setSmoothShading(bool state);
	void setNormals(bool state);
	void setMeanCurvature(bool state);
	void setEdges(bool state);
	void setPolygonDrawingMode(char mode);
	void setCameras(bool state);
	void setLighting(bool state);		
	void setTextures(bool state);
	void setSegmentIntersections(bool state);
	void setSegmentDelaunay(bool state);
	void setDebugInfo(bool state);
	void setMotionField(bool state);
	void setDebugInfoOne(bool state, int theVertexId, int theTriangleId);
	
	void downScaleInputs(int value);
	
	
	//useless sluts :)
	void setMeshTextureOne(bool state);
	void setMeshDistinctTriangles(bool state);
	void setMeshWhite(bool state);
	void setMeshWeights(bool state);
	
	void setGeometryMode(char mode);
	void setQualityMode(Mesh::QualityComputationMethod q_mode);
	
	void setFaceCullingMode(char mode);
	void setDeltas(bool state, double delta);
	void initSceneRadius();

protected:
	MVViewerRendering* gl;
	OpenGLContext* glc;
	
	int counter;
	double scaling_factor;

	virtual void draw();

	virtual void init();
	virtual void keyPressEvent(QKeyEvent *e);
	virtual void animate();

	//flags
	bool flatShading_, texturing_, lighting_, cameras_, bounding_box_, normals_, curvature_, deltas_, edges_, textures_;
 	
	bool segment_intersections_, segment_delaunay_, debug_info_, debug_info_one_, motion_field_;
	int debug_vertex_id, debug_triangle_id;

	
	GLenum geom_enum;
	double delta_scale;
	char geometry_mode;
	Mesh::QualityComputationMethod quality_mode;	
	char polygon_draw_mode;
	float bg_color[3];
};

#endif
