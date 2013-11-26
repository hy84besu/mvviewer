MVViewer
--------

Revision History
----------------
- Feb, 2012: switched to CMAKE build system (read the general README)

Info
-----
MVViewer is a base multi-camera viewer that integrates TransforMesh library. It also provides the UI base clasess for a number of derived projects (MeshMatching, MVStereo, MeshMorphing).

Library Dependencies:
---------------------
 - TransforMesh (local library provided)
 - Qt 4.x
 - libQGLViewer (http://www.libqglviewer.com/)

Compilation
-----------

In order to generate the MVViewer executable, one has to use Qt4 build system by typing : 

On Linux:
qmake MVViewer.pro; make

Use
---
Please consult the menus. They are pretty self explanatory.


Command Line
------------
By invoking ./MVViewer -help, you will see a number of command line options.

MVViewer could be useful to perform a small number of non-interactive commands. The supported non-interactive operations are listed in the "mesh_op" parameter. The input mesh(es) are mesh_op_file_1 and mesh_op_file_1, whereas the output is specified by the mesh_op_file_res parameter.


Contact
-------
Andrei Zaharescu: cooperz@gmail.com