# Matlab implementation of a Rapidly Exploring Random Tree

Run addpath_RRT.m to add repository to Matlab path.

## Basic RRT
Basic RRT implementaions in 2D and 3D.

## Simple Pendulum
Uses RRT for swing-up of a simple pendulum.
The RRT_pend.m matlab script will plot the RRT and output the control torques required.
The RRT_profiler.m matlab script shows the bottlenecks in the algorithm and generates the list of control inputs for the simulation.
The pend_sim_euler.slx is a simulink simulation of the simple pendulum.
To use the simulink simulation, run RRT_profiler OR the following first:
	>> tau = RRT_pend;
	>> control = [[0:dt:dt*length(tau)-dt]',tau'];

## Acrobot
Uses RRT for swing-up of an acrobot.
To run the simulation:
        >> control = RRT_acrobot
        Run 'acrobot_sim_v3.slx' in Simulink

## To-do
- ~~get basic RRT implementation for acrobot working~~
- partial feedback linearization
- implement in C (manually)
- test in CUDA


## Idea
- paralellize using multi-RRT (test with dual-RRT first)
