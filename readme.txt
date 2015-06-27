Matlab implementation of a Rapidly Exploring Random Tree.

The basic_RRT folder contains RRT implementaions in 2D and 3D.

The simple_pendulum folder contains the RRT implementation for the dynamics of a simple pendulum.
The RRT_pend.m matlab script will plot the RRT and output the control torques required.
The RRT_profiler.m matlab script shows the bottlenecks in the algorithm.
The pend_sim_euler.slx is a simulink simulation of the simple pendulum.
To use the simulink simulation, run RRT_profiler OR the following first:
	>> tau = RRT_pend;
	>> control = [[0:0.01:0.01*length(tau)-0.01]',tau'];

