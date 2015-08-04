function u_path = RRT_acrobot
%#codegen
% Computes the sequence of control actions needed for the swing up of
% an acrobot using a Rapidly Exploring Random Tree.
% The tree will grow from the initial state and span the phase space area,
% looking for the goal point/state.

        % acrobot paramaters
        m1 = 1;
        m2 = 1;
        l1 = 1;
        l2 = 1;
        lc1 = l1/2;
        lc2 = l2/2;
        Ic1 = (lc1*lc1)/3;
        Ic2 = (lc2*lc2)/3;
        I1 = Ic1+m1*lc1*lc1;
        I2 = Ic2+m2*lc2*lc2;
        b1 = 0;
        b2 = 0;
        g = 9.8;
       	theta1_0 = 0;
       	theta2_0 = 0;
        
	assignin('base', 'm1', m1);	% save variables into workspace
        assignin('base', 'm2', m2);
	assignin('base', 'l1', l1);
	assignin('base', 'l2', l2);
	assignin('base', 'lc1', lc1);
	assignin('base', 'lc2', lc2);
	assignin('base', 'Ic1', Ic1);
	assignin('base', 'Ic2', Ic2);
	assignin('base', 'I1', I1);
	assignin('base', 'I2', I2);
	assignin('base', 'b1', b1);
	assignin('base', 'b2', b2);
	assignin('base', 'g', g);
	assignin('base', 'theta1_0', theta1_0);
	assignin('base', 'theta2_0', theta2_0);
	
		
	x0 = [0; 0; 0; 0];	% initial state
	xG = [pi; 0; 0; 0];		% goal state
	goalRadiusSq = 9.86;	% distance from goal that is an acceptable solution
	goalBias = 0.99;
	xlimits = [-pi,pi; -10,10; -pi,pi; -10,10];	% state limits
	U = linspace(-5,5,5);		% range of control torques that can be used
	global dt;
	dt = 1;			% time interval between application of subsequent control torques
	assignin('base', 'dt', dt);
	
	u_path = 0;	% default control actions
	
	N = 200000;	% maximum number of iterations

	% pre-allocating memory
	G = repmat(x0,1,N);		% graph of states in RRT
	P = ones(1,N);			% parent states
	Ui = ones(1,N);			% control actions
	u_path = ones(1,10000);
	xbi = ones(1,10000);
	xn_c = repmat([1;1;1;1],1,length(U));

	
	
	% keep growing RRT util goal found or run out of iterations
	for n = 2:N
	        disp(n)
		% get random state
		bUseGoal = rand(1)>goalBias;
		xn = ~bUseGoal.*rand(4,1).*(xlimits(:,2)-xlimits(:,1)) + xlimits(:,1);
		
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
		temp = xn(3);
		xn(3) = mod(xn(3)+pi,2*pi)-pi;
				
		% link reachable state point to the nearest vertex in the tree
		G(:,n) = xn;
		P(1,n) = i;
		Ui(1,n) = ui;


		% if the goal was reached,
		% retrace steps from goal state to initial state
		% path displayed using red line
		test = sum((xG-xn).^2,1)
		if(test < goalRadiusSq)
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
        
        [theta1_dd, theta2_dd] = AcrobotDynamics(x(1),x(2),x(3),x(4),u);
	
	xdot = [x(2); theta1_dd; x(4); theta2_dd];
end
