#!/bin/bash

# opMode
if [ "$1" = "" ]; then
	opMode="M"
else
	opMode="$1"
fi
shift

#dataset
if [ "$1" = "" ]; then
	dataset=""
else
	dataset="$1"
fi
shift

if [ "$1" = "" ]; then
	frame1=-1
else
	frame1="$1"
fi
shift

if [ "$opMode" != "N" ]; then
	
	if [ "$1" = "" ]; then
		frame2=-1
	else
		frame2="$1"
	fi
	shift
	if [ "$1" = "" ]; then
		showresult="true"
	else
		showresult="$1"
	fi
	shift
	if [ "$1" = "" ]; then
		det_type=0
	else
		det_type="$1"
	fi
	shift
	if [ "$1" = "" ]; then
		feat_type=0
	else
		feat_type="$1"
	fi
	shift

	if [ "$1" = "" ]; then
		outfile="results.txt"
	else
		outfile="$1"
	fi
	shift

fi

if [ "$opMode" = "E" ]; then
	if [ "$1" = "" ]; then
		matches_dst_src_file=0
	else
		matches_dst_src_file="$1"
	fi
	shift
fi

if [ "$1" = "" ]; then
	noise=false
	noise_sigma_colour=0
else
	noise=true
	noise_sigma_colour=$1
fi
shift
if [ "$1" = "" ]; then
	noise=false
	noise_sigma_colour_shot=0
else
	noise=true
	noise_sigma_colour_shot=$1
fi
shift
if [ "$1" = "" ]; then
	noise_sigma_geometry=0
else
	noise_sigma_geometry=$1
fi
shift
if [ "$1" = "" ]; then
	noise_sigma_geometry_shot=0
else
	noise_sigma_geometry_shot=$1
fi
shift
if [ "$1" = "" ]; then
	noise_sigma_geometry_rotate=0
else
	noise_sigma_geometry_rotate=$1
fi
shift
if [ "$1" = "" ]; then
	noise_sigma_geometry_scale=0
else
	noise_sigma_geometry_scale=$1
fi
shift
if [ "$1" = "" ]; then
	noise_sigma_geometry_local_scale=0
else
	noise_sigma_geometry_local_scale=$1
fi
shift
if [ "$1" = "" ]; then
	noise_sigma_geometry_sampling=0
else
	noise_sigma_geometry_sampling=$1
fi
shift
if [ "$1" = "" ]; then
	noise_sigma_geometry_holes=0
else
	noise_sigma_geometry_holes=$1
fi
shift
if [ "$1" = "" ]; then
	noise_sigma_geometry_microholes=0
else
	noise_sigma_geometry_microholes=$1
fi
shift
if [ "$1" = "" ]; then
	noise_sigma_geometry_topology=0
else
	noise_sigma_geometry_topology=$1
fi
shift

#echo "gui_mode=$guimode"
#echo "frame1=$frame1"
#echo "frame2=$frame2"
#echo "outfile=$outfile"
#echo "showresult=$showresult"

#opMode=M
guimode="false"
#det_method - 0-convolutions; 1 - spectral
det_method=0
matching_2nd_ratio=0.76
axis_shift=0.8
axis_no=2
no_rings=3
detector_thresh=0.100
corner_thresh=-10
rings_as_percentage=true
choice_found=true
mesh_cleaning=false
spatial_influence=0.5
meshdesc1="0"
meshdesc2="0"
case "$dataset" in 
	'generic') 
	mesh1="$frame1"
	mesh2="$frame2"
	;;
	'generic_output') 
	mesh1="$frame1"
	mesh2="$frame2"
	matches_dst_src_file=0
	;;
	'generic_output_2') 
	mesh1="$frame1"
	mesh2="$frame2"
	matches_dst_src_file=0
	detector_thresh=130
	detector_thresh=100	
	corner_thresh=-10
	;;
	'computed') 
	mesh1="$frame1"
	mesh2="$frame2"
	meshdesc1="${det_type}"
	meshdesc2="${feat_type}"
	det_type=0
	feat_type=0
	#outfile="results.txt"
	;;	
	'cedric_flashkick') 
	resolution="lowres"
	if [ "$frame1" = "-1" ]; then 
		mesh1="./datasets_matching/cedric/flashkick/mesh_0051_0.off"
	else
		frame1=`printf '%04d_0' $frame1`
		mesh1="./datasets_matching/cedric/flashkick/mesh_$frame1.off"
	fi
	if [ "$frame2" = "-1" ]; then 
#		mesh2="./datasets_matching/cedric/flashkick/mesh_0129_0.off"
		mesh2="./datasets_matching/cedric/flashkick/mesh_0052_0.off"
	else
		frame2=`printf '%04d_0' $frame2`
		mesh2="./datasets_matching/cedric/flashkick/mesh_$frame2.off"
	fi
	axis_shift=0.8
	axis_no=2
#	det_type=1
#	feat_type=1
#	showresult=true
	no_rings=3
	mesh_cleaning=false
	spatial_influence=0.7
	;;
	'cedric_pop') 
	resolution="lowres"
	if [ "$frame1" = "-1" ]; then 
		mesh1="./datasets_matching/cedric/pop/mesh_0001_0.off"
	else
		frame1=`printf '%04d_0' $frame1`
		mesh1="./datasets_matching/cedric/pop/mesh_$frame1.off"
	fi
	if [ "$frame2" = "-1" ]; then 
		mesh2="./datasets_matching/cedric/pop/mesh_0029_0.off"
	else
		frame2=`printf '%04d_0' $frame2`
		mesh2="./datasets_matching/cedric/pop/mesh_$frame2.off"
	fi
	axis_shift=0.8
	axis_no=2
	mesh_cleaning=false
	;;

	'pop2lock') 
	resolution="lowres"
	if [ "$frame1" = "-1" ]; then 
		mesh1="./results/pop2lock/$resolution/mesh.00025.coff"
	else
		frame1=`printf '%03d' $frame1`
		mesh1="./results/pop2lock/$resolution/mesh.00$frame1.coff"
	fi
	if [ "$frame2" = "-1" ]; then 
		mesh2="./results/pop2lock/$resolution/mesh.00027.coff"
	else
		frame2=`printf '%03d' $frame2`
		mesh2="./results/pop2lock/$resolution/mesh.00$frame2.coff"
	fi
	axis_shift=0.8
	axis_no=2
	mesh_cleaning=false
	;;
	'flashkick') 
	resolution="lowres"	
	if [ "$frame1" = "-1" ]; then 
		mesh1="./results/flashkick/$resolution/mesh.00050.coff"
	else
		frame1=`printf '%03d' $frame1`
		mesh1="./results/flashkick/$resolution/mesh.00$frame1.coff"
	fi
	if [ "$frame2" = "-1" ]; then 
		mesh2="./results/flashkick/$resolution/mesh.00051.coff"
	else
		frame2=`printf '%03d' $frame2`
		mesh2="./results/flashkick/$resolution/mesh.00$frame2.coff"
	fi
	axis_shift=0.8
	axis_no=2
	mesh_cleaning=false
	;;	
	'dino') 
	mesh1="./datasets_matching/dino/dino_sparse_1.off"
#	mesh2="./results/dino_ring_1.off"
	mesh2="./datasets_matching/dino/dino_ring_shifted_sparse.coff"	
	axis_shift=0.2
	axis_no=3
	no_rings=15
	spatial_influence=0.7
	;;	
	'armadillo') 
	mesh1="./datasets_matching/armadillo/armadillo-3.off"
	mesh2="./datasets_matching/armadillo/armadillo-5.off"
	axis_shift=150
	axis_no=3
	det_type=1
	feat_type=1
	no_rings=20
	rings_as_percentage=true
	spatial_influence=0.7
	;;	
	'facesply') 
	if [ "$frame1" = "-1" ]; then 
		mesh1="./datasets_matching/faceplys/face_mesh_000306.off"
	else
		frame1=`printf '%06d' $frame1`
		mesh1="./datasets_matching/faceplys/face_mesh_$frame1.off"
	fi
	if [ "$frame2" = "-1" ]; then 
		mesh2="./datasets_matching/faceplys/face_mesh_000309.off"
		#mesh2="./datasets_matching/faceplys/face_mesh_000363.off"
	else
		frame2=`printf '%06d' $frame2`
		mesh2="./datasets_matching/faceplys/face_mesh_$frame2.off"
	fi
	no_rings=3.5
	matching_2nd_ratio=0.76
	detector_thresh=0.02
	axis_shift=200
	axis_no=3
	spatial_influence=0.8
	;;	
	'faces') 
	mesh1="./datasets_matching/faces/mesh1.coff"
	#mesh2="./datasets_matching/faces/mesh1_resample1.coff"
	mesh2="./datasets_matching/faces/mesh2.coff"
#	mesh2="./datasets_matching/faces/mesh3.coff"
	det_type=1
	feat_type=1
	axis_shift=0.4
	axis_no=3
	spatial_influence=0.7
	;;	
	'molecules') 
	mesh1="./datasets/tmp/m1d.off"
	mesh2="./datasets/tmp/m2d.off"
	det_type=1
	feat_type=1
	axis_shift=10.0
	axis_no=3
	spatial_influence=1.0
	detector_thresh=1000
	;;	
	'edmond') 
	mesh1="./datasets/tmp/bob116.off"
	mesh2="./datasets/tmp/bob120.off"
	det_type=0
	feat_type=0
	axis_shift=1.0
	axis_no=1
	spatial_influence=1.0
	matching_2nd_ratio=0.7
	no_rings=9
	detector_thresh=1000
	;;	
	'char') 
	mesh1="./datasets_matching/character/test.45.clean.off"
	mesh2="./datasets_matching/character/test.85.clean.off"
	axis_shift=0.2
	axis_no=3
	feat_type=2
	det_type=2
	;;	
	'temple') 
	mesh1="./results/temple_sparse_1.coff"
	mesh2="./results/temple_ring_1.coff"
	mesh2="./results/temple_ring_shifted_sparse.coff"	
	axis_shift=0.2
	axis_no=3	

	no_rings=3

	;;
	'ben_dance') 
	if [ "$frame1" = "-1" ]; then 
		mesh1="./datasets_matching/INRIA/ben-dance/ben_min_given_525.off"
	else
		mesh1="./datasets_matching/INRIA/ben-dance/ben_min_given_$frame1.off"
	fi
	if [ "$frame2" = "-1" ]; then 
		mesh2="./datasets_matching/INRIA/ben-dance/ben_min_given_526.off"
	else
		mesh2="./datasets_matching/INRIA/ben-dance/ben_min_given_$frame2.off"
	fi
	axis_shift=0.7
	axis_no=1
	no_rings=10
	spatial_influence=0.8
	mesh_cleaning=false
	;;
	'edmond_ball') 
	if [ "$frame1" = "-1" ]; then 
		mesh1="./results/Edmond_ball/mesh_0000.off"
	else
		mesh1="./results/Edmond_ball/mesh_0$frame1.off"
	fi
	if [ "$frame2" = "-1" ]; then 
		mesh2="./results/Edmond_ball/mesh_0030.off"
	else
		mesh2="./results/Edmond_ball/mesh_0$frame2.off"
	fi
	axis_shift=1.2
	axis_no=1
	mesh_cleaning=false
	no_rings=3
	spatial_influence=0.5	
	;;
	'mian_cheff') 
	if [ "$frame1" = "-1" ]; then 
		mesh1="./datasets_matching/mian_toys/mod-cheff-d.off"
#		mesh1="./datasets_matching/mian_toys/mod-parasaurolophus-d.off"
	else
		mesh1="./datasets_matching/mian_toys/mod-cheff-d.off"
	fi
	if [ "$frame2" = "-1" ]; then 
		mesh2="./datasets_matching/mian_toys/rs17-d.off"
		mesh2="./datasets_matching/mian_toys/rs22-d.off"
#		mesh2="./datasets_matching/mian_toys/rs23-d.off"

	else
		frame2=`printf '%d' $frame2`
		mesh2"./datasets_matching/mian_toys/rs$frame2-d.off"
	fi
	axis_shift=270
	axis_no=1
	mesh_cleaning=false
	det_type=1
	feat_type=1
	no_rings=1.8
	rings_as_percentage=true
	;;
	'mian_rhino') 
	if [ "$frame1" = "-1" ]; then 
		mesh1="./datasets_matching/mian_toys/mod-rhino-d.off"
#		mesh1="./datasets_matching/mian_toys/mod-parasaurolophus-d.off"
	else
		mesh1="./datasets_matching/mian_toys/mod-cheff-d.off"
	fi
	if [ "$frame2" = "-1" ]; then 
		mesh2="./datasets_matching/mian_toys/rs17-d.off"
		mesh2="./datasets_matching/mian_toys/rs21-d.off"

	else
		frame2=`printf '%d' $frame2`
		mesh2"./datasets_matching/mian_toys/rs$frame2-d.off"
	fi
	axis_shift=270
	axis_no=1
	mesh_cleaning=false
	det_type=1
	feat_type=1
	no_rings=1.8
	rings_as_percentage=true
	;;
	'ben_dance_surf') 
	if [ "$frame1" = "-1" ]; then 
		mesh1="./results/ben-dance/ben_dance_batch__534/output_final.off"
		meshdesc1="/scratch/monoceros/varanasi/data/backSurf/534.bsurf"		
	else
		mesh1="./results/ben-dance/ben_dance_batch__$frame1/output_final.off"
		meshdesc1="/scratch/monoceros/varanasi/data/backSurf/$frame1.bsurf"		
	fi
	if [ "$frame2" = "-1" ]; then 
		mesh2="./results/ben-dance/ben_dance_batch__535/output_final.off"
		meshdesc2="/scratch/monoceros/varanasi/data/backSurf/535.bsurf"				
	else
		mesh2="./results/ben-dance/ben_dance_batch__$frame2/output_final.off"
		meshdesc2="/scratch/monoceros/varanasi/data/backSurf/$frame2.bsurf"		
	fi
	axis_shift=0.7
	axis_no=1
	mesh_cleaning=false
	;;	
	'synth_dance') 
	if [ "$frame1" = "-1" ]; then 
		mesh1="./datasets_matching/synth_dance/Mesh-0000.off"
	else
		frame1=`printf '%03d' $frame1`
		mesh1="./datasets_scratch/synthetic/dancer/Mesh-0$frame1.off"
	fi
	if [ "$frame2" = "-1" ]; then 
		mesh2="./datasets_matching/synth_dance/Mesh-0050.off"
	else
		frame2=`printf '%03d' $frame2`
		mesh2="./datasets_scratch/synthetic/dancer/Mesh-0$frame2.off"
	fi
	axis_shift=0.1
	axis_no=2
	mesh_cleaning=false
	;;	
	'synth_dance_surf') 
	if [ "$frame1" = "-1" ]; then 
		mesh1="./datasets_scratch/synthetic/dancer/Mesh-0000.off"
		meshdesc1="./datasets_scratch/synthetic/dancer/backSurf/0.bsurf"		
	else
		meshdesc1="./datasets_scratch/synthetic/dancer/backSurf/$frame1.bsurf"
		frame1=`printf '%03d' $frame1`
		mesh1="./datasets_scratch/synthetic/dancer/Mesh-0$frame1.off"
	fi
	if [ "$frame2" = "-1" ]; then 
		mesh2="./datasets_scratch/synthetic/dancer/Mesh-0001.off"
		meshdesc1="./datasets_scratch/synthetic/dancer/backSurf/1.bsurf"
	else
		meshdesc2="./datasets_scratch/synthetic/dancer/backSurf/$frame2.bsurf"
		frame2=`printf '%03d' $frame2`
		mesh2="./datasets_scratch/synthetic/dancer/Mesh-0$frame2.off"
	fi
	axis_shift=0.1
	axis_no=2
	mesh_cleaning=false
	;;	
   	*) 
	echo "-------------------------------------------------------------------------------------------"
	echo "Command: run_matching_dataset.sh [opmode] [dataset_name] [frame1] [frame2] [showresults]"
	echo "-------------------------------------------------------------------------------------------"
	echo "Operation mode: M - matching; E - evaluation"
	echo "Available Datasets:"
	echo "	- dino temple"
	echo "	- pop2lock flashkick ben_dance ben_dance_surf synth_dance synth_dance_surf"
	echo "	- cedric_pop cedric_flashkick faces"
	echo "	- molecules"
	echo "Special Datasets:"
	echo "	- generic: allows one to load two specific meshes"
	echo "	- computed: allows one to load two specific meshes as well as the computed descriptors (det_desc files)"	
	echo "-------------------------------------------------------------------------------------------"
	choice_found=false		
esac

if [ $choice_found = true ]; then
	
#	det_type=1
#	feat_type=1
	
	./MeshMatching \
	-op_mode $opMode \
	-src_mesh_file $mesh1 \
		-src_mesh_desc_file $meshdesc1 \
	-dst_mesh_file $mesh2 \
		-dst_mesh_desc_file $meshdesc2 \
		-matches_dst_src_file $matches_dst_src_file \
	-gui $guimode -save_output_file $outfile \
	\
	-detector_type $det_type \
		-detector_method $det_method \
		-detector_thresh $detector_thresh \
		-scale_space true \
		-no_scales 3 \
		-corner_thresh $corner_thresh \
		-non_max_sup true \
	-feature_type $feat_type \
		-feat_uses_det_scale true \
		-no_rings $no_rings \
		-rings_as_percentage $rings_as_percentage \
		-no_bins_centroid 36 \
		-no_bins_groups 4 \
		-spatial_influence $spatial_influence\
	-matching_2nd_ratio $matching_2nd_ratio \
		-groundtruth 2 \
		-save_output_format sum \
	-noise $noise \
		-noise_sigma_colour $noise_sigma_colour \
		-noise_sigma_colour_shotnoise $noise_sigma_colour_shot \
		-noise_sigma_geom_noise $noise_sigma_geometry \
		-noise_sigma_geom_shotnoise $noise_sigma_geometry_shot \
		-noise_sigma_geom_rotate $noise_sigma_geometry_rotate \
		-noise_sigma_geom_scale $noise_sigma_geometry_scale \
		-noise_sigma_geom_local_scale $noise_sigma_geometry_local_scale \
		-noise_sigma_geom_sampling $noise_sigma_geometry_sampling \
		-noise_sigma_geom_holes $noise_sigma_geometry_holes \
		-noise_sigma_geom_micro_holes $noise_sigma_geometry_microholes \
		-noise_sigma_geom_topology $noise_sigma_geometry_topology 

	if [ "$showresult" = "true" -a "$opMode" = "M" ]; then
		if [ "$noise" = "true" ]; then
			./scripts/apartVectorOff.pl mesh1_noised.coff $mesh2 $outfile $axis_shift $axis_no > $outfile.world
		else
			./scripts/apartVectorOff.pl $mesh1 $mesh2 $outfile $axis_shift $axis_no > $outfile.world
		fi
		geomview $outfile.world
	fi
#	echo "choice found"
fi
