#!/bin/bash

#datasets=( dino_sparse temple_sparse dino_ring  temple_ring )
#datasets=( ben_dance_435 ben_dance_436)

dataset="$1"
range1=$2
range2=$3
filemask="$4"

echo "running $dataset btw [$range1;$range2] with mask $filemask"
reps=$range2-$range1+1
if [ "`uname`" = "Darwin" ]; then
datasets=(`jot $reps $range1 $range2`)
else
datasets=(`seq $range1 $range2`)
fi


for (( i = 1 ; i < ${#datasets[@]} ; i++ ))
do
	frame1=${datasets[$i-1]};
#	frame1=1;
	frame2=${datasets[$i]};
	outfile=`printf $filemask $frame1 $frame2`
	echo "Running set $frame1 $frame2 with output $outfile";
	./scripts/run_matching_dataset.sh $dataset false $frame1 $frame2 $outfile false

done



