#!/bin/bash


# srcFolder
if [ "$1" = "" ]; then
	srcFolder=./out
else
	srcFolder="$1"
fi
shift

# dstFolder
if [ "$1" = "" ]; then
	dstFolder=./out_scrambled
else
	dstFolder="$1"
fi
shift

mkdir $dstFolder

echo "Processing:"
for x in `ls $srcFolder/*.off`; do
	echo " - $x"
	filename="`basename $x`"
	basefilename="`echo $filename | sed 's/.off//'`"
	./scrambleCoff $srcFolder/$filename $srcFolder/$basefilename.matches $dstFolder/$filename $dstFolder/$basefilename.matches
done

echo "Copying input meshes"

rm -f $dstFolder/*-null.*
for x in `ls $srcFolder/*-null.off`; do
	echo " - $x"
	filename="`basename $x`"
	cp $srcFolder/$filename $dstFolder/$filename
done
