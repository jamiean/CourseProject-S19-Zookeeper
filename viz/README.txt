Mac Instructions
    1. To set up, run:

        $bash setup_mac.sh
    2. To plot only graph input run:

        $bash viz_local.sh <input_file_name> <"mst"|"tsp">

        use "mst" option to only show edges between pokemon
        which are on both on the sea, both on land, or on
        the coast
    
    3.  To plot graph input and program output, run:

            $bash viz_local.sh <input_file_name> <output_file_name> <"mst"|"tsp">

Ubuntu Instructions:
    Same as Mac, but run

    $bash setup_ubuntu.sh 
    
    to set up
             
CAEN Instructions:
    1. To set up, run:

        $bash setup_caen.sh (this may take a while)
 
    2. To plot only graph input run:

        $bash viz_caen.sh <input_file_name> <"mst"|"tsp">

        use "mst" option to only show edges between pokemon
        which are on both on the sea, both on land, or on
        the coast
    
    3.  To plot graph input and program output, run:

            $bash viz_caen.sh <input_file_name> <output_file_name> <"mst"|"tsp">

Windows Instructions:
    1. Install the following packages manually:
        1) https://www.python.org/ftp/python/2.7.8/python-2.7.8.msi
        2) https://bootstrap.pypa.io/get-pip.py
        3) http://www.microsoft.com/en-us/download/confirmation.aspx?id=44266

    2. From a Command Prompt, Run:
        setup_win

    3. To plot only graph input run:

        viz_win <input_file_name> <"mst"|"tsp">

        use "mst" option to only show edges between pokemon
        which are on both on the sea, both on land, or on
        the coast
    
    4.  To plot graph input and program output, run:

            viz_win <input_file_name> <output_file_name> <"mst"|"tsp">

Example
	After you have everything set up, you can test it with the sample
	input/output files provided.  Use your proper viz_* script, for example:
	
		$bash viz_caen.sh input.txt output-mst.txt mst
	
		viz_win input.txt output-tsp.txt tsp
