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
The RRT_acrobot.m matlab script outputs the control torques required, without showing a plot.
The RRT_acrobot_fromC.m matlab script saves the control torques from 'build-RRT_acrobot_C-Desktop_Qt_5_3_GCC_64bit-Debug/data.txt' into the matlab workspace. The data.txt file is generated from the C version of the RRT algorithm (source in 'RRT_acrobot_c').
The acrobot_sim_v3.slx is a simulink simulation of the acrobot.
To use the simulink simulation, run either of the following first:
        
        >> control = RRT_acrobot
                OR
        >> RRT_acrobot_fromC;      (after running the C code)

Not sure if PFL implementation is working properly (probably needs task space PFL).

### To-do
- ~~get basic RRT implementation for acrobot working~~
- ~~implement in C (manually)~~
- test in CUDA
- partial feedback linearization (need to work on task space PFL)


### Idea
- parallelize using multi-RRT (test with dual-RRT first)
