function control = RRT_pend
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
	l = 1;	    % length of pendulum link
	b = 0.1;	% damping factor
	theta_0 = -pi/2;    % initial angular position
	
	assignin('base', 'm', m);	% save variables into workspace
	assignin('base', 'g', g);
	assignin('base', 'l', l);
	assignin('base', 'b', b);
	assignin('base', 'theta_0', theta_0);
	
	x0 = [-pi/2; 0];	% initial state; angle position measured from x-axis
	xG = [-pi/2; 6];		% goal state

	xlimits = [-pi,pi; -10,10];	% state limits
	
	U = linspace(-5,5,20);		% range of control torques that can be used
	
	dt = 0.01;			% time interval between application of subsequent control torques
	assignin('base', 'dt', dt);
	
	N = 50000;	% maximum number of iterations

	% pre-allocating memory
	G = repmat(x0,1,N);		% stores a graph of states in RRT
	P = ones(1,N);			% stores index of parent states
	Ui = ones(1,N);			% stores index of control actions in U
	u_path = ones(1,1000);  % stores sequence of control actions (solution to problem)
	xbi = 1;                % index used for traceback
	xn_c = repmat([1;1],1,length(U));   % stores temporary achievable states from a particular vertex

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
    	xn = rand(2,1).*(xlimits(:,2)-xlimits(:,1)) + xlimits(:,1);

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
		if( (xn(1) > pi) || (xn(1) < -pi) )
        		xn(1) = mod(xn(1)+pi,2*pi)-pi;
        end
		
		% plot new RRT branch
		if(abs(xn(1)-temp) < pi)
			line([G(1,i),xn(1)],[G(2,i),xn(2)],'Color','b');
		end
		
		% link reachable state point to the nearest vertex in the tree
		G(:,n) = xn;
		P(n) = i;
		Ui(n) = ui;

		% for higher values of n, only update plot every 250 iteration (speeds up animation)
		if(n<100 || mod(n,250)==1)
			drawnow;
		end

		% if the goal was reached, stop growing tree
		if((xn(1) <= xG(1)+0.1) && (xn(1) >= xG(1)-0.1))
		    if((xn(2) <= xG(2)+0.5) && (xn(2) >= xG(2)-0.5))			
			    break;
			end
		end
		
	end
	
	if(n == N)
		title('Simulation complete (goal not found; ran out of iterations)');
	else		% retrace steps from goal state to initial state
	    title('Simulation complete (goal found)');
        xbi = n;
        index = 1;

        % retrace control actions and solution trajectory
		% path displayed using red line
        while(xbi ~= 1)
	        xx = [G(1,xbi),G(1,P(xbi))];
	        if abs(xx(2)-xx(1))<pi
		        line([G(1,xbi),G(1,P(xbi))],[G(2,xbi),G(2,P(xbi))],'Color','r','LineWidth',2);
	        end
	
            u_path(index) = U( Ui(xbi) );
            index = index+1;
	        xbi = P(xbi);
        end
        
        index2 = 1;
       	u_path_final = ones(1,index-1);
        while(index > 1)
            u_path_final(index2) = u_path(index-1);
            index = index-1;
            index2 = index2+1;
        end
        
        control = [[0:dt:dt*length(u_path_final)-dt]',u_path_final'];

        drawnow;
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
