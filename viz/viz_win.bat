@rem Usage:
@rem    bash viz_caen.sh <input_file> <output_file> <"mst"|"tsp">
@rem    bash viz_caen.sh <input_file> <"mst"|"tsp">
@rem
@rem Example:
@rem    bash viz_caen.sh input-1.txt output-1.txt mst
@rem    bash viz_caen.sh input-1.txt tsp
@rem
@rem
@del input.png
@del output.png

@IF "%~3"=="" GOTO onearg
@echo vizgraph.py -i %1 -o %2 -m %3
@vizgraph.py -i %1 -o %2 -m %3
@input.png
@output.png
@goto done
:onearg
@echo vizgraph.py -i %1 -m %3
@vizgraph.py -i %1 -m %3
@input.png
:done