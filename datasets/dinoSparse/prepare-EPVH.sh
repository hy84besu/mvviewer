#!/bin/bash
i=0
imax=15
rm -rf EPVH
mkdir EPVH
while [ $i -le $imax ]
do

frame=`printf %.4d $i`

echo "Frame = " $frame
cat camera$frame.txt >> EPVH/calib
echo " " >> EPVH/calib
convert dinoSR$frame.png  EPVH/dino-img-$frame.pgm
convert dinoR$frame-mask.png EPVH/dino-sil-$frame.pgm
#convert dinoSR$frame.png dinoR$frame-mask.png +matte -compose CopyOpacity -composite dinoSR$frame-cleaned.tif
 i=$(($i+1))
done
