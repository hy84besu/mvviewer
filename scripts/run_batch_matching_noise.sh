#!/bin/bash

#datasets=( dino_sparse temple_sparse dino_ring  temple_ring )
#datasets=( ben_dance_435 ben_dance_436)

dataset="$1"
steps=$2
filemask="$3"

echo "running $dataset with $steps error steps and with mask $filemask"
if [ "`uname`" = "Darwin" ]; then
sigma=(`jot $steps 0.00 0.50`)
else
steps=`echo "scale=2;0.5/$steps"|bc`
sigma=(`seq 0.00 $steps 0.50`)
fi


for (( i = 0 ; i < ${#sigma[@]} ; i++ ))
do
	for (( j = 0 ; j < ${#sigma[@]} ; j++ ))
	do

		noise_colour=${sigma[$i]};
		noise_geometry=${sigma[$j]};		
		outfile=`printf $filemask $noise_colour $noise_geometry`
		echo "****************************************************************************************************"
		echo "Running set with noise_colour=$noise_colour and noise_geometry=$noise_geometry with output $outfile";
		./scripts/run_matching_dataset.sh $dataset false -1 -1 $outfile false 0 0 $noise_colour $noise_geometry
	done
done



