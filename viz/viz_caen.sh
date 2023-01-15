#!/bin/bash
#
# Usage:
#    bash viz_caen.sh <input_file> <output_file> <"mst"|"tsp">
#    bash viz_caen.sh <input_file> <"mst"|"tsp">
#
# Example:
#    bash viz_caen.sh input-1.txt output-1.txt mst
#    bash viz_caen.sh input-1.txt tsp
#
#
. virtualenv_config.sh

# rm input.png output.png

. "$VIRTUALENV_DIR"/bin/activate

echo . "$VIRTUALENV_DIR"/bin/activate
if [[ "$#" == 2 ]]; then
    echo python vizgraph.py -i $1 -m $2
    python vizgraph.py -i $1 -m $2
elif [[ "$#" == 3 ]]; then
    echo python vizgraph.py -i $1 -o $2 -m $3
    python vizgraph.py -i $1 -o $2 -m $3
else
    cat <<HELP
To plot only graph input, run:

    $0 <input-file-name> <"mst"|"tsp">

To plot graph input and program output, run:

    $0 <input-file-name> <output-file-name> <"mst"|"tsp">

HELP
    exit 1
fi

mkdir -p ~/Public/html/
cp input.png output.png ~/Public/html/
echo "Your visualizations are saved at:" *.png "and also accessible at your personal webpage hosted by the University of Michigan"
echo "http://www-personal.umich.edu/~$USER/input.png"
echo "http://www-personal.umich.edu/~$USER/output.png"
