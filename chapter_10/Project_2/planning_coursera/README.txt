The RRT algorithm has been implemented to solve the motion planning problem.

The following assumptions were done:
- The probability of selecting the goal node when performing sampling is 10% and is editable by the user only inside the code
- When creating the local planner, x_new has been selected equal to x_samp, in this way a node in the direction of x_samp can be easily created and at the same time the algorithm can converge.

Once the code stored in the "RRT_algorithm.py" script generated the "nodes.csv" and "edges.csv" files, the "Astar_search.py" script was used to implement an A* search to search the graph and generate the "path.csv" file. 

The solution files have then been passed to CoppeliaSim inside Scene5 to have a graphical representation of the motion planning problem.
