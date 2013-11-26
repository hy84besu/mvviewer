#!/bin/bash

if [ "$1" = "" ]; then
	dataset=""
else
	dataset="$1"
fi
shift
if [ "$1" = "" ]; then
	guimode="false"
else
	guimode="$1"
fi
shift
if [ "$1" = "" ]; then
	frame1=-1
else
	frame1="$1"
fi
shift
if [ "$1" = "" ]; then
	frame2=-1
else
	frame2="$1"
fi
shift
if [ "$1" = "" ]; then
	outfile="./matches.MHOG"
else
	outfile="$1"
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
	noise=false
	noise_sigma_colour=0.1
else
	noise=true
	noise_sigma_colour=$1
fi
shift
if [ "$1" = "" ]; then
	noise_sigma_geometry=0.1
else
	noise_sigma_geometry=$1
fi
shift

#echo "gui_mode=$guimode"
#echo "frame1=$frame1"
#echo "frame2=$frame2"
#echo "outfile=$outfile"
#echo "showresult=$showresult"

choice_found=true
mesh_cleaning=false
meshdesc1="0"
meshdesc2="0"
case "$dataset" in 
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
	mesh2="./results/dino_ring_1.off"
	mesh2="./datasets_matching/dino/dino_ring_shifted_sparse.coff"	
	axis_shift=0.2
	axis_no=3
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
#	./scripts/run_matching.sh   false  0.2 3	
	;;
	'ben_dance') 
	if [ "$frame1" = "-1" ]; then 
		mesh1="./results/ben-dance/ben_dance_batch__534/output_final.off"
	else
		mesh1="./results/ben-dance/ben_dance_batch__$frame1/output_final.off"
	fi
	if [ "$frame2" = "-1" ]; then 
		mesh2="./results/ben-dance/ben_dance_batch__535/output_final.off"
	else
		mesh2="./results/ben-dance/ben_dance_batch__$frame2/output_final.off"
	fi
	axis_shift=0.7
	axis_no=1
	mesh_cleaning=false
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
	echo "Command: run_matching_dataset.sh [dataset_name] [use_gui] [frame1] [frame2] [outfile] [showresults]"
	echo "-------------------------------------------------------------------------------------------"
	echo "Available Datasets:"
	echo "	- dino temple"
	echo "	- pop2lock flashkick ben_dance ben_dance_surf synth_dance synth_dance_surf"
	echo "-------------------------------------------------------------------------------------------"
	choice_found=false		
esac

if [ $choice_found = true ]; then
	
#	det_type=1
#	feat_type=1
	
	#./scripts/run_matching.sh $mesh1 $mesh2 $guimode $axis_shift $axis_no $outfile
	#echo ./MeshMatching -src_mesh_file $mesh1 -dst_mesh_file $mesh2 -no_rings 14 -spatial_influence 0.5 -matching_2nd_ratio 0.70 -gui $guimode -save_output_file $outfile -non_max_sup false
	
	if [ $mesh_cleaning = true ]; then	
		./MeshMatching -src_mesh_file $mesh1 -dst_mesh_file $mesh2 -no_rings 14 -spatial_influence 0.5 -matching_2nd_ratio 0.70 -gui $guimode -save_output_file $outfile.det -groundtruth 1 -save_output_format det
		./MatchCleaning -feat_in_file $outfile.det -feat_out_file $outfile -angle_variance 0.4 -angle_thresh 0.2
	else
	./MeshMatching -src_mesh_file $mesh1 -src_mesh_desc_file $meshdesc1 -dst_mesh_file $mesh2 -dst_mesh_desc_file $meshdesc2 -no_bins_centroid 36 -no_bins_groups 4 -spatial_influence 0.3 -matching_2nd_ratio 0.70 -gui $guimode -save_output_file $outfile -non_max_sup true -feature_type $feat_type -detector_type $det_type -detector_thresh 0.05  -groundtruth 2 -save_output_format sum -noise $noise -noise_sigma_colour $noise_sigma_colour -noise_sigma_geometry $noise_sigma_geometry -scale_space false -no_rings 1 -rings_as_percentage true -corner_thresh -10 #-features_only
	fi
	if [ "$showresult" = "true" ]; then
		./scripts/apartVectorOff.pl $mesh1 $mesh2 $outfile $axis_shift $axis_no > $outfile.world		
		geomview $outfile.world
	fi
#	echo "choice found"
fi
