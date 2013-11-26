#!/bin/bash

if [ "$1" = "" ]; then
	DetType=0
else
	DetType="$1"
fi
shift
if [ "$1" = "" ]; then
	FeatType=0
else
	FeatType="$1"
fi
shift
	
if [ "$1" = "" ]; then
	FolderIn="./datasets_matching/ZHR11-scalar1-orig"
else
	FolderIn="$1"
fi
shift
if [ "$1" = "" ]; then
	FolderOut="./out_eval/run-det$DetType-desc$FeatType"
else
	FolderOut="$1"
fi
shift


echo "Creating Destination Folder $outFile"
mkdir $FolderOut

noMeshes=3
for (( i=1; i<=noMeshes; i++ ))
do
	echo "========== -> Mesh $i <- =================="
	./scripts/run_operation_scalar_func1_db.sh B $FolderIn $FolderOut `printf '%03d' $i` $DetType $FeatType
done

./scripts/generate_tables.sh $FolderOut 1 $FeatType
./scripts/generate_tables.sh $FolderOut 2 $FeatType