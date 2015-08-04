function u_path = RRT_pend
%#codegen
% Computes the sequence of control actions needed for the swing up of
% a simple pendulum using a Rapidly Exploring Random Tree.
% The tree will grow from the initial state and span the phase space area,
% looking for the goal point/state (indicated by the red marker).
% If the goal state was located, a red line traces back the sequence of states used
% to reach the goal state.

	% pendulum parameters
	m = 1;		% mass
	g = 9.8;	% acceleration due to gravity
	l = 1;	% length of pendulum link
	b = 0.1;	% damping factor
	theta_0 = -pi/2;
	
	assignin('base', 'm', m);	% save variables into workspace
	assignin('base', 'g', g);
	assignin('base', 'l', l);
	assignin('base', 'b', b);
	assignin('base', 'theta_0', theta_0);
	
	x0 = [-pi/2; 0];	% initial state; angle position measured from x-axis
	xG = [pi/2; 0];		% goal state
	goalRadiusSq = 0.02;	% distance from goal that is an acceptable solution
	goalBias = 0.99;
	xlimits = [-pi,pi; -10,10];	% state limits
	U = linspace(-5,5,20);		% range of control torques that can be used
	global dt;
	dt = 0.1;			% time interval between application of subsequent control torques
	assignin('base', 'dt', dt);
	
	u_path = 0;	% default control actions
	
	N = 20000;	% maximum number of iterations

	% pre-allocating memory
	G = repmat(x0,1,N);		% graph of states in RRT
	P = ones(1,N);			% parent states
	Ui = ones(1,N);			% control actions
	u_path = ones(1,1000);
	xbi = ones(1,1000);
	xn_c = repmat([1;1],1,length(U));

	
	
	% keep growing RRT util goal found or run out of iterations
	for n = 2:N
	
		% get random state
		bUseGoal = rand(1)>goalBias;
		xn = ~bUseGoal.*rand(2,1).*(xlimits(:,2)-xlimits(:,1)) + xlimits(:,1);
		
		% find distances between that state point and every vertex in RRT
		dsq = euclidianDistSquare(xn,G(:,1:n));
		
		% select RRT vertex closest to the state point
		[~,i] = min(dsq);
		
		% from the closest RRT vertex, compute all the states that can be reached,
		% given the pendulum dynamics and available torques
		for ui = 1:length(U)
			xn_c(:,ui) = G(:,i) + dt*dynamics(G(:,i),U(ui));
		end
		
		% select the closest reachable state point
		dsq = euclidianDistSquare(xn,xn_c);
		[~,ui] = min(dsq);
		xn = xn_c(:,ui);
		
		% if angular position is greater than pi rads, wrap around
		temp = xn(1);
		xn(1) = mod(xn(1)+pi,2*pi)-pi;
				
		% link reachable state point to the nearest vertex in the tree
		G(:,n) = xn;
		P(1,n) = i;
		Ui(1,n) = ui;


		% if the goal was reached,
		% retrace steps from goal state to initial state
		% path displayed using red line
		if(sum((xG-xn).^2,1) < goalRadiusSq)
			xbi = n;
			
			% retrace control actions and solution trajectory
			u_path = [];
			while(xbi(1) ~= 1)
				xx = [G(1,xbi(1)),G(1,P(xbi(1)))];
				
				u_path = [U(Ui(xbi(1))),u_path];
				xbi = [P(xbi(1)),xbi];
			end
			
			break;
		end
	end
end

function d = euclidianDistSquare(xs,X)
% compute the squares of the euclidian distance from xs to every point in X
	Y = repmat(xs,1,size(X,2));
	d = sum((X-Y).^2,1);
end

function xdot = dynamics(x,u)
	% pendulum parameters
	m = 1;		% mass
	g = 9.8;	% acceleration due to gravity
	l = 1;		% length of pendulum link
	I = m*l*l;	% rotational inertia
	b = 0.1;	% damping factor
	
	% pendulum state space equation
	xdot = [x(2,:); (u-m*g*l*sin((pi/2)-x(1,:))-b*x(2,:))./I];
end
