#!/bin/bash
if [ "$1" = "" ]; then
	Operation="none"
else
	Operation="$1"
fi
shift
if [ "$1" = "" ]; then
	FolderIn="./out"
else
	FolderIn="$1"
fi
shift
if [ "$1" = "" ]; then
	FolderOut="./out"
else
	FolderOut="$1"
fi
shift
if [ "$1" = "" ]; then
	BasefileIn="$FolderIn/mesh-001"
	BasefileOut="$FolderOut/mesh-001"	
else
	BasefileIn="$FolderIn/mesh-$1"
	BasefileOut="$FolderOut/mesh-$1"	
fi
shift
if [ "$1" = "" ]; then
	detType=0
else
	detType="$1"
fi
shift
if [ "$1" = "" ]; then
	featType=0
else
	featType="$1"
fi
shift

if [ "$Operation" = "none" ]; then
	echo "Parameters: Operation InputFolder OutputFolder [MeshID] [DetectionType] [FeatureType]"
	echo "  - Operation: G - Generate database; F - Compute Features; E - Evaluate Features; B - Both compute and evaluate features"
	echo "  - InputFolder: the folder where the input meshes and matches are"
	echo "  - OutputFolder: the folder where the output features and/or results will be"
	echo "  - MeshID: the ID of the mesh (by default is 1)"
	echo "  - DetectionType: When computing the features, which scalar function to use for feature detection"
	echo "  - FeatureType: When computing the features, which scalar function to use for the descriptor"
	exit
fi

mesh1="$BasefileIn-null.off"

genNoise=0
genFeatures=0
evalFeatures=0

if [ "$Operation" = "G" ]; then
	genNoise=1
elif [ "$Operation" = "F" ]; then
	genFeatures=1
elif [ "$Operation" = "E" ]; then
	evalFeatures=1
elif [ "$Operation" = "B" ]; then
	genFeatures=1
	evalFeatures=1
fi

if [ -d $FolderOut ]; then
	echo "Output directory exists. Good to know.";
else
	echo "Creating output directory";
	mkdir $FolderOut
fi

featNullFile="$BasefileOut-null"
./scripts/run_matching_dataset.sh F generic $mesh1 $mesh1 false $detType $featType $featNullFile

function evaluate {
	if [ "$1" = "" ]; then
		runNoise=1
	else
		runNoise=$1
	fi
	if [ "$2" = "" ]; then
		runFeatEval=1
	else
		runFeatEval=$2
	fi


	for i in ${!vNoise[*]}
	do
		noise=${vNoise[$i]} 
		mesh2="${BasefileIn}-$noiseType-$i.off"
		matches="${BasefileIn}-$noiseType-$i.matches"
		noiseModel=`echo $noiseTemplate|sed s/noise/$noise/`

		if [ "$runNoise" = "1" ]; then
			echo "######## Generate Noise mesh $mesh2 ###############"
			./scripts/run_matching_dataset.sh N generic $mesh1 $mesh2 $noiseModel 
			cp ./mesh1_noised.coff $mesh2
			cp ./mesh1_matches.txt $matches
		fi

		if [ -f $mesh2 ]; then

			if [ "$genFeatures" = "1" ]; then
				echo "######## Computing features for $mesh2 ###############"
				featNoiseFile="${BasefileOut}-$noiseType-$i"
				./scripts/run_matching_dataset.sh F generic $mesh2 $mesh2 false $detType $featType $featNoiseFile
			fi				

			if [ "$evalFeatures" = "1" ]; then
				echo "######## Evaluating $mesh2 ###############"
				results="${BasefileOut}-$noiseType-$i.results"
				./scripts/run_matching_dataset.sh E computed $mesh1 $mesh2 false $featNullFile.det_desc $featNoiseFile.det_desc $results $matches
				echo $mesh2 `cat $results` >> $FolderOut/all_results.txt
			fi
		else
			echo "Could not locate $mesh2"
		fi
	done
}

noiseType="color-noise"
noiseTemplate="noise 0 0 0 0 0 0 0 0 0"
vNoise=([1]=0.002 [2]=0.005 [3]=0.01 [4]=0.02 [5]=0.05);
evaluate $genNoise

noiseType="color-shot-noise"
noiseTemplate="0 noise 0 0.0 0 0 0 0 0 0 0"
vNoise=([1]=0.002 [2]=0.005 [3]=0.01 [4]=0.02 [5]=0.05);
evaluate $genNoise

noiseType="geom-noise"
noiseTemplate="0 0.0 noise 0 0 0 0 0 0 0"
vNoise=([1]=0.1 [2]=0.2 [3]=0.3 [4]=0.4 [5]=0.5);
evaluate $genNoise

noiseType="geom-shot-noise"
noiseTemplate="0 0.0 0 noise 0 0 0 0 0 0 0"
vNoise=([1]=0.002 [2]=0.005 [3]=0.01 [4]=0.02 [5]=0.05);
evaluate $genNoise

noiseType="geom-rotate"
noiseTemplate="0 0.0 0 0 noise 0 0 0 0 0 0"
vNoise=([1]=0.1 [2]=0.2 [3]=0.3 [4]=0.4 [5]=0.5);
evaluate $genNoise

noiseType="geom-scale"
noiseTemplate="0 0.0 0 0 0 noise 0 0 0 0 0"
vNoise=([1]=0.25 [2]=0.438 [3]=0.625 [4]=0.8125 [5]=1.0);
evaluate $genNoise

noiseType="geom-local-scale"
noiseTemplate="0 0.0 0 0 0 0 noise 0 0 0 0"
vNoise=([1]=0.1 [2]=0.2 [3]=0.3 [4]=0.4 [5]=0.5);
evaluate $genNoise

noiseType="geom-sampling"
noiseTemplate="0 0.0 0 0 0 0 0 noise 0 0 0"
vNoise=([1]=0.1 [2]=0.2 [3]=0.3 [4]=0.4 [5]=0.5);
evaluate $genNoise

noiseType="geom-holes"
noiseTemplate="0 0.0 0 0 0 0 0 0 noise 0 0"
vNoise=([1]=0.1 [2]=0.2 [3]=0.3 [4]=0.4 [5]=0.5);
evaluate $genNoise

noiseType="geom-micro-holes"
noiseTemplate="0 0.0 0 0 0 0 0 0 0 noise 0"
vNoise=([1]=0.2 [2]=0.4 [3]=0.6 [4]=0.8 [5]=1.0);
evaluate $genNoise

noiseType="geom-topology"
noiseTemplate="0 0.0 0 0 0 0 0 0 0 0 noise"
vNoise=([1]=0.1 [2]=0.2 [3]=0.3 [4]=0.4 [5]=0.5);
evaluate $genNoise

noiseType="geom-isometry"
noiseTemplate="0 0.0 0 0 0 0 0 0 0 0 noise"
vNoise=([1]=0.1 [2]=0.2 [3]=0.3 [4]=0.4 [5]=0.5);
evaluate 0