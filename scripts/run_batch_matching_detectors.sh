#!/bin/bash

#datasets=( dino_sparse temple_sparse dino_ring  temple_ring )
#datasets=( ben_dance_435 ben_dance_436)

dataset="$1"
frame1=-1
frame2=-1
filemask="$2"

det_type_list=(0 1)
feat_type_list=(0 1 3)

echo "Running batch detectors for $dataset with mask $filemask"
for (( d = 0 ; d < ${#det_type_list[@]} ; d++ ))
do

	for (( f = 0 ; f < ${#feat_type_list[@]} ; f++ ))
	do
		det_type=${det_type_list[$d]};
		feat_type=${feat_type_list[$f]};		
		outfile=`printf $filemask $det_type $feat_type`
#		echo "Running set with output $outfile";
		./scripts/run_matching_dataset.sh $dataset false $frame1 $frame2  $outfile false $det_type $feat_type
	done
done



