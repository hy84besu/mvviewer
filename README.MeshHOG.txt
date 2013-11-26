MeshHOG and MeshDOG Library
---------------------------

Revision History
----------------
- Feb, 2012: switched to CMAKE build system (read the general README)

Info
-----
MeshHOG and MeshDOG are implementations of the work described in
http://perception.inrialpes.fr/publication.php3?bibtex=ZBH12


Library Dependencies:
---------------------
- TransforMesh (also a local project) - uses  the Polyhedron data-structure; it could  re-factored to be using the CGAL Polyhedron class and only some basic mesh operations only (mean curvature computation, convolution,etc).

Please check the associated README.TMESH as well.

Compilation
-----------

Read the compilation notes for TransforMesh and adapt them to this project. The name of the project is
MeshMatching.pro


How to use the library
----------------------

MeshMatching executable contains the code that does performs feature detection (MeshDOG), feature description (MeshHOG) and matching / evaluation. The command supports a lot of command line options (try MeshMatching -h to see what they are).

The first parameter "op_mode", refers to the mode of operation:
'F' - Features:  extract features and descriptors of one provided input mesh
'M' - Matching: extract features and descriptors of two input meshes and attempt to match them
'E' - Evaluation:  evaluate extracted features & descriptors against provided groundtruth data
'N' - Noise: noise the input mesh with one of the possible noise types

If one wants to perform feature detection & description for one mesh only, then the command line option "-op_mode F" should be used. 
BE AWARE that the $save_output_file file generated contains more than just the descriptor on each line.

The output file format (.det_desc) is the following:
- the first line in the header file represents the number of descriptors (ND) and the descriptor size (DS);
- on each subsequent line there is: I DESCRIPTOR HSV COORDS NORMAL, where I is the vertex index (starting from 0);  DESCRIPTOR are the following DS numbers; HSV, COORDS, NORMAL have each 3 values;

If one wants to perform feature detection and matching, then the command line option "-op_mode M" should be used.
Run the executable with "-h" to see all the command line options. Also, check the scripts folder:

./scripts/run_matching_dataset.sh
and
./scripts/run_matching.sh
for examples of scripts that perform matching on different data sets.

Geomview can be used to view the generated .world files.

Examples
--------

"Dino" : 
- command line :./scripts/run_matching_dataset.sh M dino
- info: two separate 3-D reconstructions (obtained using the method described in Zaharescu et al., "TransforMesh: a topology-adaptive mesh-based approach to surface evolution", ACCV2007) of the same rigid model viewed by different camera sets (input images taken from the Middleburry Multi-View Stereo).

"Synth Dance":
- command line:  ./scripts/run_matching_dataset.sh M synth_dance
- info: synthetically generated data.


Use and Referencing
-------------------
Please reference our work appropriately when using this library. I am very interested to hear about people using this library. Do not hesitate to contact me to let me know. The current library is free to use for academic purposes. It is released under GPL licence.

http://perception.inrialpes.fr/Publications/2012/ZBH12/
 

Contact
-------
Andrei Zaharescu: cooperz@gmail.com