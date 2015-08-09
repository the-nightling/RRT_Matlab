function functionProfiles = RRT_profiler
	profile on -history
	global dt;
	tau = RRT_pend;
	p = profile('info');
	
	control = [[0:dt:dt*length(tau)-dt]',tau'];
	assignin('base', 'control', control);
	
	functionName = [];
	functionTime = [p.FunctionTable.TotalTime]';
	for n = 1:size(p.FunctionTable,1)
		functionName = [functionName,cellstr(p.FunctionTable(n).FunctionName)];
	end
	functionName = functionName';
	functionProfiles = flipud(sortrows([num2cell(functionTime),functionName]));
	clear p n functionName functionTime tau;
