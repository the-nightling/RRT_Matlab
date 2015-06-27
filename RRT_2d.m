function RRT_2d()
%
% Matlab implementation of a Rapidly Exploring Random Tree in 2D
% The tree will grow from the initial point/state and span the phase space area,
% looking for the goal point/state (indicated by a red circle).
% Constraints, represented in black, are states that cannot be taken. Rectangular
% regions have been used but any other formula can be used to 'select out' states.
% If the goal state was located, a red line traces back the sequence of states used
% to reach the goal state.
%
% The following variables can be changed as per required:
% - plot dimensions
% - initial point
% - goal point
% - goal radius (determines how close a point has to be to the goal point
%			 to be validated as a success)
% - growth factor (ranges from 0 to 1; sets limit for length of new branch; 
%			a higher value  will explore the space using fewer iterations
%			a lower value will give better sensitivity
% - number of maximum iterations (1000 iterations takes about a minute to complete in
%			the worst case; i.e. goal not found)
% - delay (to obtain an animated plot; A value lower than 0.005 won't show the animation;
%			The pause function can be commented out to make the algorithm run at full speed)
% - constraints (can be added in the local checkConstraint() function)
%

	% set phase space dimensions
	xMax = 10;
	yMax = 10;
	
	% set initial state
	xInit = 0;
	yInit = 0;
	
	% set goal state
	xGoal = 8;
	yGoal = 8;
	goalRadius = 0.2;
	
	% set limit for length of new branch
	growthFactor = 0.4;
	
	% set maximum iterations allowable
	noOfIterations = 1000;
	
	% set delay to view animation
	delay = 0.005;

	% initialize figure
	figure
	plot(xInit,yInit)
	plot(xGoal,yGoal,'ro')
	hold on
	grid on
	xlabel('x')
	ylabel('y')
	axis([0,xMax,0,yMax]);
	
	% initialize variables
	nodeIndex = 1;
	goalFound = 0;
	initialNode.xCoord = xInit;
	initialNode.yCoord = yInit;
	initialNode.parentNode = 0;
	goalNode.xCoord = xGoal;
	goalNode.yCoord = yGoal;
	nodeArray(nodeIndex).xCoord = initialNode.xCoord;
	nodeArray(nodeIndex).yCoord = initialNode.yCoord;
	nodeArray(nodeIndex).parentNode = initialNode.parentNode;
	nodeIndex = nodeIndex + 1;

	% keep growing RRT util goal found or run out of iterations
	for iteration = [1:noOfIterations]
	
		% get random point in phase space
		randomNode.xCoord = rand() * xMax;
		randomNode.yCoord = rand() * yMax;
		
		% find nearest vertex of RRT to random point by checking with all vertices of RRT
		% [bottleneck of algorithm; increases exponentially with every new added vertex]
		nearestNode = nodeArray(1);
		for node = nodeArray
			if(getDistanceBetween(randomNode, node) <= getDistanceBetween(randomNode, nearestNode))
				nearestNode = node;
			end
		end
	
		% limit the distance of the new point from the tree by adding a point
		% that is in the same direction but closer (distance determined by growth factor)
		hyp = getDistanceBetween(randomNode, nearestNode);
		if(hyp < growthFactor)
			newNode = randomNode;
		else
			x = randomNode.xCoord - nearestNode.xCoord;
			y = randomNode.yCoord - nearestNode.yCoord;
			angleAlpha = acos(x/hyp);
			angleBeta = acos(y/hyp);
			newNode.xCoord = nearestNode.xCoord + growthFactor*cos(angleAlpha);
			newNode.yCoord = nearestNode.yCoord + growthFactor*cos(angleBeta);
		end
		
		% discard new point if it coincides with a constraint
		if(checkConstraint(newNode))
			continue;
		end
		
		% link new node with nearest vertex/node in the tree
		newNode.parentNode = nearestNode;
		nodeArray(nodeIndex).xCoord = newNode.xCoord;
		nodeArray(nodeIndex).yCoord = newNode.yCoord;
		nodeArray(nodeIndex).parentNode = newNode.parentNode;
		nodeIndex = nodeIndex + 1;

		% plot new RRT branch
		plot([newNode.xCoord nearestNode.xCoord],[newNode.yCoord nearestNode.yCoord]);
		drawnow;
		% if goal reached, end iterations
		if(getDistanceBetween(newNode, goalNode) <= goalRadius)
			goalFound = 1;
			break;
		end
		
	    %pause(delay);   
	end
	
	% if the goal was reached,
	% retrace steps from goal state to initial state
	% path displayed using red line
	if(goalFound)
		title('Simulation complete (goal found)')
		tracebackNode = nodeArray(nodeIndex-1);
		while(isstruct(tracebackNode.parentNode))
			plot([tracebackNode.xCoord tracebackNode.parentNode.xCoord],[tracebackNode.yCoord tracebackNode.parentNode.yCoord],'r');
			tracebackNode = tracebackNode.parentNode;
		end
	else
		title('Simulation complete (goal not found; ran out of iterations)')
	end
	
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% LOCAL FUNCTIONS											%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% returns euclidian distance between two points
function [distance] = getDistanceBetween(point1, point2)
	distance = sqrt((point1.xCoord-point2.xCoord)^2 + (point1.yCoord-point2.yCoord)^2);
end

% checks whether a point is in a forbidden area
function [constrained] = checkConstraint(node)
	if((node.xCoord > 2) && (node.xCoord < 5) && (node.yCoord > 2) && (node.yCoord < 5))
		% plot forbidden area in black (rectangle)
		plot([2 5],[2 2],'k')
		plot([2 2],[2 5],'k')
		plot([2 5],[5 5],'k')
		plot([5 5],[2 5],'k')
		constrained = 1;
	elseif((node.xCoord > 7) && (node.xCoord < 8) && (node.yCoord > 6) && (node.yCoord < 7))
		% plot forbidden area in black (rectangle)
		plot([7 8],[6 6],'k')
		plot([7 7],[6 7],'k')
		plot([7 8],[7 7],'k')
		plot([8 8],[6 7],'k')
		constrained = 1;
	elseif((node.xCoord > 5) && (node.xCoord < 6) && (node.yCoord > 6) && (node.yCoord < 7))
		% plot forbidden area in black (rectangle)
		plot([5 6],[6 6],'k')
		plot([5 5],[6 7],'k')
		plot([5 6],[7 7],'k')
		plot([6 6],[6 7],'k')
		constrained = 1;
	else
		constrained = 0;
	end
end
