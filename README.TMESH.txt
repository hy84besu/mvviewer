TransforMesh Library
--------------------

Info
-----
TransforMesh is a library that implements a mesh based scheme for mesh based evolution dealing with topological chances and self-intersections.

Paper link: http://perception.inrialpes.fr/Publications/2007/ZBH07/

Library Dependencies:
---------------------
 - CGAL (3.3.1): http://www.cgal.org/
 - CImg: http://cimg.sourceforge.net/
 - CEP: http://cgm.cs.mcgill.ca/~stever/CGAL/


Out of the dependencies specified, only CGAL has to be installed. The other libraries are provided in the src/other_libs directory in the source form.

Compilation
-----------

In order to generate the TransforMesh lib, one has to use Qt4 build system by typing : 

On Linux:
qmake TransforMesh.pro; make

The project itself does not depend on Qt.

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
and ensuring that all edges are within the specified interval, eliminates degenerate triangles.

The example MeshMorphing shows how a mesh can be gradually evolved over time.

Use and Referencing
-------------------
Please reference our work appropriately when using this library. I am very interested to hear about the people
using this library. Do not hesitate to contact me to let me know. The current library is free to use for academic purposes. It will be shortly released under GPL like licence.

http://perception.inrialpes.fr/Publications/2007/ZBH07/
 

Contact
-------
Andrei Zaharescu: cooperz@gmail.com