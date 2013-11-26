#!/bin/bash
#cd /Users/andreiz/Documents/Projects/INRIA/TransforMesh/matlab/scripts

inputMesh=$1;
inputFunction=$2;
outputFile=$3;
noEigs=100;

noScales=$4;
noSubscales=$5;
baseSigma=$6;

rm -f $outputFile

echo "cd /Users/andreiz/Documents/Projects/INRIA/TransforMesh/matlab/scripts; script_scalespace('$inputMesh','$inputFunction','$outputFile', $noEigs, $noScales, $noSubscales, $baseSigma,0); exit" > ./tmp_matlab_script.txt


/Applications/MATLAB_R2009b.app/bin/matlab -maci -nojvm -nosplash -nodesktop < tmp_matlab_script.txt > tmp_matlab_output.txt