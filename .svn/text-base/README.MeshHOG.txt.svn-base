MeshHOG and MeshDOG Library
---------------------------

Revision History
----------------
- Dec 16th, 2009: Added examples section, that shows a couple of running examples.

Info
-----
MeshHOG and MeshDOG are implementations of the work described in
Paper link: http://perception.inrialpes.fr/publication.php3?bibtex=ZBVH09

Library Dependencies:
---------------------
- TransforMesh (also a local project) - uses  the Polyhedron data-structure; it could be easily re-written to be using the CGAL Polyhedron class only 
- MVViewer (also a local project) - only for UI - not really used in practice, since the application is meant to be run from the command line mode

Compilation
-----------

Read the compilation notes for TransforMesh and adapt them to this project. The name of the project is
MeshMatching.pro


How to use the library
----------------------

MeshMatching contains the code that does feature detection (MeshDOG), feature description (MeshHOG) and feature comparison. If one wants to perform feature detection & description for one mesh only, then the command line option "-features_only true" should be used.

Run the executable with "-h" to see all the command line options. Also, check the scripts folder:

./scripts/run_matching_dataset.sh
and
./scripts/run_matching.sh

for examples of scripts that perform batch on different data sets.

Geomview can be used to view the generated .world files.

Examples
--------

"Dino" : 
- command line :./scripts/run_matching_dataset.sh din
- info: two separate 3-D reconstructions (obtained using the method described in Zaharescu et al., "TransforMesh: a topology-adaptive mesh-based approach to surface evolution", ACCV2007) of the same rigid model viewed by different camera sets (input images taken from the Middleburry Multi-View Stereo).

"Synth Dance":
- command line:  ./scripts/run_matching_dataset.sh synth_dance
- info: synthetically generated data.


Use and Referencing
-------------------
Please reference our work appropriately when using this library. I am very interested to hear about the people using this library. Do not hesitate to contact me to let me know. The current library is free to use for academic purposes. It is released under GPL licence.

http://perception.inrialpes.fr/Publications/2009/ZBVH09/
 

Contact
-------
Andrei Zaharescu: cooperz@gmail.com
