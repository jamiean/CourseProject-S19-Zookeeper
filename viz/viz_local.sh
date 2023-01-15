#!/bin/bash
# 
# Usage:
#    bash viz_local.sh <input_file> <output_file>
#    bash viz_local.sh <input_file>
#
# Example:
#    bash viz_local.sh input-1.txt output-1.txt
#    bash viz_local.sh input-1.txt
#
#
rm input.png output.png
if [ ! -z $3 ]
then
	echo python vizgraph.py -i $1 -o $2 -m $3
	python vizgraph.py -i $1 -o $2 -m $3
else
	echo python vizgraph.py -i $1 -m $2
	python vizgraph.py -i $1 -m $2
fi
open *.png || eog *.png
echo "Your visualizations are saved at:" *.png
