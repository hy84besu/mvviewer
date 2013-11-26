#!/bin/bash
if [ "$1" = "" ]; then
	basefolder="./out"
else
	basefolder="$1"
fi
shift
if [ "$1" = "" ]; then
	resColumn=1
else
	resColumn="$1"
fi
shift
if [ "$1" = "" ]; then
	featType=0
else
	featType="$1"
fi
shift


output="latex" # text or latex


# the levels of noise
vTotal=([1]=0 [2]=0 [3]=0 [4]=0 [5]=0);
totalCount=0

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
		results="${basefolder}/*-$noiseType-$i.results"
		vNoise[$i]=`cat $results|cut -d' ' -f$resColumn|awk '{sum+=$1}END{print sum/NR, " ", sum, " ",  NR}'|cut -d' ' -f1`
		vTotal[$i]=`echo ${vNoise[$i]} + ${vTotal[$i]} | bc -l`
	done
	let totalCount+=1
	display
}

function displayText {
printf "$displayNoiseType :\t"
for i in ${!vNoise[*]}
do
	printf "%0.2f \t" ${vNoise[$i]}
done
echo ""	
}


function displayLatex {
	if [ "$displayNoiseType" = "Average" ]; then
		echo -n "\textbf{$displayNoiseType} & "
	else
		echo -n "$displayNoiseType & "
	fi
	for i in ${!vNoise[*]}
	do
		printf "%0.2f " ${vNoise[$i]}
		if [ "$i" -ne "${#vNoise[@]}" ]; then
			printf "& "
		fi
	done
	echo "\\\\"	
}


function display {
	if [ "$output" = "latex" ]; then
		displayLatex
	else
		displayText
	fi
}

function displayHeader {
	if [ "$output" = "latex" ]; then

		echo "\begin{table}[!htbp]"
		echo "\centering"
		echo "\small"		
		echo "\begin{tabular}{rrrrrr}"
		echo "& \multicolumn{5}{c}{\textbf{Strength}} \\\\"
		echo "\cline{2-6}"
		echo "\textbf{Transform.} & 1 & \$< \$ 2 & \$ < \$ 3  & \$<\$ 4 & \$<\$ 5 \\\\"
		echo "\toprule"
	fi
}

function displayScalarFunction {
	if [ "$featType" = "0" ]; then
		echo -n "(photometric)"
	elif [ "$featType" = "1" ]; then
		echo -n "(mean curvature)"
	else
		echo -n "(Gaussian curvature)"
	fi
}

function displayFooter {
	if [ "$output" = "latex" ]; then
		echo "\hline"
		echo "\end{tabular}"
		
		if [ "$resColumn" = "1" ]; then
			echo -n "\caption{Repeatability of MeshDOG "
			displayScalarFunction
			echo "}"
		else
			echo -n "\caption{Robustness of MeshHOG "
			displayScalarFunction
			echo "}"

		fi
		echo "\label{table:change_label}"
		echo "\end{table}"
	fi
}


vNoise=([1]=0.002 [2]=0.005 [3]=0.01 [4]=0.02 [5]=0.05);
displayHeader

noiseType="color-noise"
displayNoiseType="Color Noise"
evaluate

noiseType="color-shot-noise"
displayNoiseType="Color Shot Noise"
evaluate

noiseType="geom-noise"
displayNoiseType="Geometry Noise"
evaluate

noiseType="geom-shot-noise"
displayNoiseType="Geometry Shot Noise"
evaluate

noiseType="geom-rotate"
displayNoiseType="Rotation"
evaluate

noiseType="geom-scale"
displayNoiseType="Scale"
evaluate

noiseType="geom-local-scale"
displayNoiseType="Local Scale"
evaluate

noiseType="geom-sampling"
displayNoiseType="Sampling"
evaluate

noiseType="geom-holes"
displayNoiseType="Holes"
evaluate

noiseType="geom-micro-holes"
displayNoiseType="Micro-Holes"
evaluate


noiseType="geom-topology"
displayNoiseType="Topology"
evaluate


noiseType="geom-isometry"
displayNoiseType="Isometry + Noise"
evaluate 0 1

if [ "$output" = "latex" ]; then
	echo "\hline"
fi

# compute the average
for i in ${!vTotal[*]}
do
	vNoise[$i]=`echo ${vTotal[$i]} / $totalCount | bc -l`
done
displayNoiseType="Average"
display

displayFooter
	

