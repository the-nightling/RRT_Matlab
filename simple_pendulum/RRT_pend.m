function u_path = RRT_pend
%
% Computes the sequence of control actions needed for the swing up of
% a simple pendulum using a Rapidly Exploring Random Tree.
% The tree will grow from the initial state and span the phase space area,
% looking for the goal point/state (indicated by the red marker).
% If the goal state was located, a red line traces back the sequence of states used
% to reach the goal state.

	% pendulum parameters
	global m g l I b dt
	m = 1;		% mass
	g = 9.8;	% acceleration due to gravity
	l = 0.5;	% length of pendulum link
	I = m*l*l;	% rotational inertia
	b = 0.1;	% damping factor
	theta_0 = -pi/2;
	
	assignin('base', 'm', m);	% save variables into workspace
	assignin('base', 'g', g);
	assignin('base', 'l', l);
	assignin('base', 'b', b);
	assignin('base', 'theta_0', theta_0);
	
	x0 = [-pi/2; 0];	% initial state; angle position measured from x-axis
	xG = [pi/2; 0];		% goal state
	goalRadiusSq = 0.05;	% distance from goal that is an acceptable solution
	goalBias = 0.99;
	xlimits = [-pi,pi; -5,5];	% state limits
	U = linspace(-5,5,20);		% range of control torques that can be used
	dt = 0.005;			% time interval between application of subsequent control torques
	
	u_path = 0;	% default control actions
	x_path = 0;	% default state trajectory
	
	N = 20000;	% maximum number of iterations

	% pre-allocating memory
	G = repmat(x0,1,N);		% graph of states in RRT
	P = ones(1,N);			% parent states
	Ui = ones(1,N);			% control actions

	% setup plot
	figure(1);
	hold off;
	plot(x0(1),x0(2),'b.','MarkerSize',30);	% initial state in blue
	hold on;
	plot(xG(1),xG(2),'r.','MarkerSize',30);	% goal state in red
	grid on;

	axis([xlimits(1,:),xlimits(2,:)]);
	xlabel('Angular position [rad]');
	ylabel('Angular velocity [rad/s]');
	
	set(gca,'XTick',-pi:pi/4:pi,'XTickLabel',{'-pi','-3pi/4','-pi/2','-pi/4','0','pi/4','pi/2','3pi/4','pi'});
	
	% keep growing RRT util goal found or run out of iterations
	for n = 2:N
	
		% get random state
		bUseGoal = rand(1)>goalBias;
		xn = ~bUseGoal.*rand(2,1).*(xlimits(:,2)-xlimits(:,1)) + xlimits(:,1);
		
		% find distances between that state point and every vertex in RRT
		dsq = euclidianDistSquare(xn,G);
		
		% select RRT vertex closest to the state point
		[y,i] = min(dsq);
		
		% from the closest RRT vertex, compute all the states that can be reached,
		% given the pendulum dynamics and available torques
		for ui = 1:length(U)
			xn_c(:,ui) = G(:,i) + dt*dynamics(G(:,i),U(ui));
		end
		
		% select the closest reachable state point
		dsq = euclidianDistSquare(xn,xn_c);
		[y,ui] = min(dsq);
		xn = xn_c(:,ui);
		
		% if angular position is greater than pi rads, wrap around
		temp = xn(1);
		xn(1) = mod(xn(1)+pi,2*pi)-pi;
		
		% plot new RRT branch
		if(abs(xn(1)-temp) < pi)
			line([G(1,i),xn(1)],[G(2,i),xn(2)],'Color','b');
		end
		
		% link reachable state point to the nearest vertex in the tree
		G(:,n) = xn;
		P(1,n) = i;
		Ui(1,n) = ui;

		% for higer values of n, only update plot every 250 iteration (speeds up animation)
		if(n<100 || mod(n,250)==1)
			drawnow;
		end

		% if the goal was reached,
		% retrace steps from goal state to initial state
		% path displayed using red line
		if(euclidianDistSquare(xG,xn) < goalRadiusSq)
			title('Simulation complete (goal found)');
			xbi = n;
			
			% retrace control actions and solution trajectory
			u_path = [];
			while(xbi(1) ~= 1)
				xx = [G(1,xbi(1)),G(1,P(xbi(1)))];
				if abs(xx(2)-xx(1))<pi
					line([G(1,xbi(1)),G(1,P(xbi(1)))],[G(2,xbi(1)),G(2,P(xbi(1)))],'Color','r','LineWidth',2);
				end
				
				u_path = [U(Ui(xbi(1))),u_path];
				xbi = [P(xbi(1)),xbi];
			end
			
			drawnow;
			
			x_path = [];
			for ti = 1:length(xbi);
				x_path = [x_path,G(:,xbi(ti))];
			end
			
			break;
		end
		
		n = n+1;
		if(n == N+1)
			title('Simulation complete (goal not found; ran out of iterations)');
		end
	end
end

function d = euclidianDistSquare(xs,X)
% compute the squares of the euclidian distance from xs to every point in X
	d = sum((X-repmat(xs,1,size(X,2))).^2,1);
end

function xdot = dynamics(x,u)
	% pendulum parameters
	global m g l I b;
	% pendulum state space equation
	xdot = [x(2,:); (u-m*g*l*sin((pi/2)-x(1,:))-b*x(2,:))./I];
end
