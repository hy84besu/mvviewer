TransforMesh Library
--------------------

Revision History
----------------
- Feb, 2012: switched to CMAKE build system (read the general README)

Info
-----
TransforMesh is a library that implements a mesh based scheme for mesh based evolution dealing with topological chances and self-intersections.

Paper link: http://perception.inrialpes.fr/Publications/2007/ZBH07/

Library Dependencies:
---------------------
 - CGAL (3.5): http://www.cgal.org/
 - CImg: http://cimg.sourceforge.net/
 - CEP: http://cgm.cs.mcgill.ca/~stever/CGAL/


Out of the dependencies specified, only CGAL has to be installed. The other libraries are provided in the src/other_libs directory in the source form.

Compilation
-----------

In order to generate the TransforMesh lib, one has to use Qt4 build system by typing : 

On Linux:
qmake TransforMesh.pro; make

The project itself does not depend on Qt.

Troubleshooting:

1) I got a number of complains from users trying to compile, getting  "Mesh.cpp:(.text+0x26f3): undefined reference to `dgelss_'". That is because you do not have CGAL properly configured to use BLAS.
If you look in TransforMesh.files, just remove "WITH_CGAL_BLAS" from the defines. That is only needed in the dependent project "MeshMatching" and it is not used to remove self-intersections. However, if you are using the MeshDOG/MeshHOG detector / descriptor, I recommend that you fix the error, since solving for the scalar function gradient using the linear system gives better results.

Example
-------

We have provided an example application that uses TransforMesh, called MeshMorphing. In order to compile it, use:

qmake MeshMorphing.pro; make


How to use the library
----------------------

In order to interface with the TransforMesh library, a "Mesh" class is being exposed.
Meshes in the OFF file format can be loaded and saved using the loadFormat and saveFormat methods.
The method that invokes TransforMesh is removeSelfIntersections(). It is the user's responsibility to
make sure that the mesh does not contain degenerate triangles. This can be easily handled by invoking
the method named ensureEdgeSize(low_thresholf,high_threshold), which, on top of optimizing the vertex valence
and ensuring that all edges are within the specified interval, eliminates degenerate triangles. It is also the user's responsibility to make sure that the provided mesh is does not contain any boundary conditions (i.e. vertices, edges or facets coincide). That can easily be disambiguated via simulation of simplicity (a small random perturbation). 

The example MeshMorphing shows how a mesh can be gradually evolved over time.

Also, see the MVViewer project, that provides a mesh viewer user interface as well as hook-ups to invoke TransforMesh.
The functions of interest can be accessed from the Mesh menu button:
- Compute Self-Intersections (it detects the number of intersections)
- Remove Self-Intersections (it invokes TransforMesh)
- Ensure Edge-Size - it ensures that all the edges are within a specified edge size range (bear in mind that when invoking TransforMesh, it is the user's responsibility to make sure that the provided mesh does not contain any degeneracies).

In addition, MVViewer has a number of command line options for simple mesh processing (without the user interface).
Self intersection removal can be invoked via mesh_op parameter (clean1 or clean2). For example:

./MVViewer -mesh_op clean2 -mesh_op_file_1 input.off  -mesh_op_file_res output.off



Use and Referencing
-------------------
Please reference our work appropriately when using this library. I am very interested to hear about the people using this library. Do not hesitate to contact me to let me know. The current library is free to use for academic purposes. It will be shortly released under GPL like licence.

http://perception.inrialpes.fr/Publications/2010/ZBH10/ 

Contact
-------
Andrei Zaharescu: cooperz@gmail.com
