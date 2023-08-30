## scampi_factor_graph_optimization
For generating the results and error a jupyter notebook script is provided. At first for saving the FK and IK outputs we need to make a senario and feed some initial pos for master code to save the results. 
The real data contain 1000 frame and all the data is saved in `matlab_data`, from these data the initial poses for all these frame saved as csv file and are located in . Next with compiling and running `master.cpp`, using these initial poses, the inverse and forward results for fatorgraph optimization will be saved in `data_from_cpp_to_python`
