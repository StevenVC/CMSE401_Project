# CMSE401_Project
Author: Steven VanCamp

To learn about this project reference ```CMSE401_Project_WriteUp_vancam25.pdf```

The source code is contained in ```src/```. The main source file being compiled for in this project is ```src/optimizer_parallel.cpp```. 

```src/optimizer.cpp``` is depreciated.

# Example Code
Clone directory

Run the following commands ```bash example.sb``` or ```sbatch example.sb``` if supported

```example.sb``` runs the main optimization algorithm 9 times each with a different thread count with the following order ```[1 2 4 6 8 10 12 14 16]```. Update line 33 in ```example.sb``` if you wish to change this behavior.

This will generate files containing the results for each execution in a directory named ```example_results/```

To see the results for the various executions look at the files ```alg_sol_*.txt```. These will contain some information about the execution of the program.

Additional files will be made ```g_best_hist_*.txt```. These will contain two coloumns 

    | global best fit value at iteration i | execution time at iteration i |