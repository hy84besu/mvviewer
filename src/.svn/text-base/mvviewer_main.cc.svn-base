#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include "CImg_mod.h"
#include <string.h>

#include "MVWindow.h"
#include "MVData.h"
#include <qapplication.h>

int main(int argc, char** argv)
{
	
cimg_usage("Multi-Camera Mesh Viewer\n Params: run the program with the -h option in order to view them.");
    const int no_cameras = cimg_option("-no_cameras",0,"Number of cameras");
	
    const char *camera_files = cimg_option("-camera_files",(const char*)NULL,"Camera files (i.e. camera%04d.txt)");
    const char *image_files = cimg_option("-image_files",(const char*)NULL,"Image files (i.e. img%04d.png)");	
    const char *mesh_file = cimg_option("-mesh_file","./datasets/dinoSparse/model.off","Mesh file");		

	const char *mesh_op = cimg_option("-mesh_op","none","Operations: none, union, diff");
	const char *mesh_op_file_1 = cimg_option("-mesh_op_file_1","./datasets/dinoSparse/model.off","Mesh file - operand 1");
    const char *mesh_op_file_2 = cimg_option("-mesh_op_file_2","./datasets/dinoSparse/model.off","Mesh file");
    const char *mesh_op_file_res = cimg_option("-mesh_op_file_res","./results/tmp.off","Mesh file");

	
	if (strcmp(mesh_op,"none")!=0) {
		Mesh m1,m2;
		m1.loadFormat(mesh_op_file_1,"OFF");
		m2.loadFormat(mesh_op_file_2,"OFF");
		if (strcmp(mesh_op,"diff")==0) {
			cout << "Performing mesh difference" << endl;
			m2.invert();
			m1.unionWith(m2);
			m1.saveFormat(mesh_op_file_res,"OFF");			
		}
		if (strcmp(mesh_op,"union")==0) {
			cout << "Performing mesh union" << endl;			
			m1.unionWith(m2);
			m1.saveFormat(mesh_op_file_res,"OFF");
		}
		return 0;
	}
	
    if (cimg_option("-h",(const char *)0,0)) return 0;
 	if (argc<5) {
		cerr << "Error: Not enough arguments, try the -h option" << endl;
		//		return 1;
	}
	
	// Read command lines arguments.
	QApplication application(argc,argv);
	
	MVData data;
	data.loadData(no_cameras,0,1,NULL,camera_files,mesh_file,image_files,NULL);
	
	// Instantiate the viewer.
	MVWindow v;
	v.viewer->setMVData(&data);
	
	// Make the viewer window visible on screen.
	v.show();
	
#if QT_VERSION < 0x040000
	// Set the viewer as the application main widget.
	application.setMainWidget(&v);
#endif
	
	// Run main loop.
	return application.exec();
}
