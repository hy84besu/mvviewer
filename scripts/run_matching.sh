#!/bin/bash

mesh1=$1
mesh2=$2


if [ "$3" = "" ]; then
	GUI_MODE="false"
else
	GUI_MODE="$3"
fi

if [ "$4" = "" ]; then
	shift=0.8
else
	shift="$4"
fi

if [ "$5" = "" ]; then
	axis=2
else
	axis="$5"
fi

if [ "$6" = "" ]; then
	outfile="./matches.txt"
else
	outfile="$6"
fi

echo "gui_mode=$GUI_MODE"
echo "shift=$shift"
echo "axis=$axis"
echo "outfile=$outfile"
	
echo ./MeshMatching -src_mesh_file $mesh1 -dst_mesh_file $mesh2 -no_rings 14 -spatial_influence 0.2 -matching_2nd_ratio 0.75 -gui $GUI_MODE -save_output_file $outfile
./MeshMatching -src_mesh_file $mesh1 -dst_mesh_file $mesh2 -no_rings 14 -spatial_influence 0.5 -matching_2nd_ratio 0.75 -gui $GUI_MODE -save_output_file $outfile -feature_type 0 -detector_type 0 -non_max_sup false
./scripts/apartVectorOff.pl $mesh1 $mesh2 $outfile $shift $axis > $outfile.world
geomview $outfile.world

