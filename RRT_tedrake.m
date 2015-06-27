function RRT_tedrake

	% makes the basic, pretty RRT plot
	x_start = [0.0;0.0];
	x_goal = [0.8;0.8];
	goal_bias = 0.05;
	max_edge_length = 0.02;
	step_length = 0.002;

	N = 10000;
	V = repmat(x_start,1,N);
	parent = nan(1,N-1);
	xtraj = [];

	hold off;
	plot(x_start(1),x_start(2),'b.','MarkerSize',20);
	hold on;
	plot(x_goal(1),x_goal(2),'r.','MarkerSize',20);
	axis([0,1,0,1]);
	grid on;

	n=2;

	while n <= N
		try_goal = rand < goal_bias;
		if try_goal
			xs = x_goal;
		else
			xs = rand(2,1);
			if checkConstraints(xs)
				continue;
			end
		end
	
		d = euclidianDistance(V(:,1:n-1),xs);
		[dmin,imin] = min(d);
	
		if dmin > max_edge_length
			xs = V(:,imin)+(max_edge_length/dmin)*(xs - V(:,imin));
			dmin = max_edge_length;
			try_goal = false;
		end
	
		if dmin > step_length
			num_intermediate_samples = 2 + ceil(dmin/step_length);
			xss = linspacevec(V(:,imin),xs,num_intermediate_samples);
			% check constraints for each point
			valid = true;
			for i = 2: num_intermediate_samples-1
				if checkConstraints(xss(:,i))
					valid = false;
					break;
				end
			end
			if ~valid
				continue;
			end
		end
	
		line([V(1,imin),xs(1)],[V(2,imin),xs(2)],'Color','b');
		V(:,n) = xs;
		parent(n-1) = imin;
	
		drawnow;
	
		if(try_goal)
			path = n;
			while path(1) > 1
				path = [parent(path(1)-1),path];
				line([V(1,path(2)),V(1,path(1))],[V(2,path(2)),V(2,path(1))],'Color','r');
				drawnow;
			end
			return;
		end
		
		n = n + 1;
	end
end



function d = euclidianDistance(X,xs)
	d = sqrt(sum((X-repmat(xs,1,size(X,2))).^2,1));
end

function y = linspacevec(d1,d2,n)
	y = repmat(d1,1,n) + (d2-d1)*(0:n-1)/(n-1);
end

% checks whether a point is in a forbidden area
function [constrained] = checkConstraints(node)
	if((node(1) > 0.2) && (node(1) < 0.5) && (node(2) > 0.2) && (node(2) < 0.5))
		% plot forbidden area in black (rectangle)
		plot([0.2 0.5],[0.2 0.2],'k')
		plot([0.2 0.2],[0.2 0.5],'k')
		plot([0.2 0.5],[0.5 0.5],'k')
		plot([0.5 0.5],[0.2 0.5],'k')
		constrained = 1;
	elseif((node(1) > 0.7) && (node(1) < 0.8) && (node(2) > 0.6) && (node(2) < 0.7))
		% plot forbidden area in black (rectangle)
		plot([0.7 0.8],[0.6 0.6],'k')
		plot([0.7 0.7],[0.6 0.7],'k')
		plot([0.7 0.8],[0.7 0.7],'k')
		plot([0.8 0.8],[0.6 0.7],'k')
		constrained = 1;
	elseif((node(1) > 0.5) && (node(1) < 0.6) && (node(2) > 0.6) && (node(2) < 0.7))
		% plot forbidden area in black (rectangle)
		plot([0.5 0.6],[0.6 0.6],'k')
		plot([0.5 0.5],[0.6 0.7],'k')
		plot([0.5 0.6],[0.7 0.7],'k')
		plot([0.6 0.6],[0.6 0.7],'k')
		constrained = 1;
	else
		constrained = 0;
	end
end
