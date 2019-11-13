Submission for Formal Methods for CPS Programming Assignment 2.
Based on 'The SMT-Based Automatic Road Network Generation in Vehicle Simulation Environment', Kim et. al.
Meriel Stein (ms7nk)

NOTE: 
Code runtime is on a scale of several hours. Bottleneck has been determined to be the local curvature constraints in road_seg_gen().
Roman has looked over it and deemed the implementation of the algorithms from the paper to be correct, the visualization to be correct, and the use of Z3 to be correct. Running this code on Roman's laptop, Meriel's mac laptop, and an Ubuntu desktop suggested a similar runtime. All had upwards of 8GB memory and sufficient disk space.

N.b. Solver is passed as a parameter instead of a global as a precaution to ensure control flow. After runtime issue became apparent, this was done on the off chance that residual assumptions remain in global after s.reset().

FILE MANIFEST:
coordinates.txt -- Unity coordinates scaled by 5.
coordinates_scale30.txt -- original Unity coordinates scaled by 30.
road_network2.png -- visualization of original Unity coordinates using matplotlib.
road_network_gen_output2.txt -- output of code. Includes assumptions for each road segment and runtime for each road segment.
z3_project.py -- source code with algorithms implementations from SMT paper.
