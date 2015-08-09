## Matlab implementation of a Rapidly Exploring Random Tree

Run addpath_RRT.m to add repository to Matlab path.

### Basic RRT
Basic RRT implementations in 2D and 3D.

### Simple Pendulum
Uses RRT for swing-up of a simple pendulum.
The RRT_pend.m matlab script will plot the RRT and output the control torques required.
The RRT_pend_novisuals.m matlab script outputs the control torques required, without showing a plot.
The RRT_pend_fromC.m matlab script saves the control torques from 'build-RRT_pend_C-Desktop_Qt_5_3_GCC_64bit-Debug/data.txt' into the matlab workspace. The data.txt file is generated from the C version of the RRT algorithm (source in 'RRT_pend_c').
The pend_sim_euler.slx is a simulink simulation of the simple pendulum.
To use the simulink simulation, run either of the following first:
        
        >> control = RRT_pend;
                OR
        >> control = RRT_pend_novisuals;
                OR
        >> RRT_pend_fromC;      (after running the C code)

### Acrobot
Uses RRT for swing-up of an acrobot.
To run the simulation:
        
        >> control = RRT_acrobot
        Run 'acrobot_sim_v3.slx' in Simulink

Not sure if PFL implementation is working properly (probably needs task space PFL).

### To-do
- ~~get basic RRT implementation for acrobot working~~
- partial feedback linearization
- implement in C (manually) -> done for simple pendulum
- test in CUDA


### Idea
- parallelize using multi-RRT (test with dual-RRT first)
