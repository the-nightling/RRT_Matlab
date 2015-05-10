Matlab implementation of a Rapidly Exploring Random Tree in 2D and 3D.

The tree will grow from the initial point/state and span the phase space area, looking for the goal point/state (indicated by a red circle).

Constraints, represented in black, are states that cannot be taken. Rectangular and cuboidal regions have been used but any other formula can be used to 'select out' states.

If the goal state was located, a red line traces back the sequence of states used to reach the goal state.

The following variables can be changed as per required:
- plot dimensions
- initial point
- goal point
- goal radius (determines how close a point has to be to the goal point to be validated as a success)
- growth factor (ranges from 0 to 1; sets limit for length of new branch; a higher value  will explore the space using fewer iterations; a lower value will give better sensitivity)
- number of maximum iterations (1000 iterations takes about a minute and a half to complete in the worst case; i.e. goal not found)
- delay (to obtain an animated plot; A value lower than 0.005 won't show the animation; the pause function can be commented out to make the algorithm run at full speed)
- constraints (can be added in the local checkConstraint() function)

When using RRT_3d, the view() Matlab function can be used to inspect the 3D plot:
view(0,90) - xy view
view(0,0)  - xz view
view(90,0) - yz view
view(3)    - 3D view
