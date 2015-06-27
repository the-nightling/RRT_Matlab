profile on -history
tau = RRT_pend;
control = [[0:0.01:0.01*length(tau)-0.01]',tau'];
p = profile('info');
functionName = [];
functionTime = [p.FunctionTable.TotalTime]';
for n = 1:size(p.FunctionTable,1)
	functionName = [functionName,cellstr(p.FunctionTable(n).FunctionName)];
end
functionName = functionName';
functionProfiles = flipud(sortrows([num2cell(functionTime),functionName]))
clear p n functionName functionTime tau;
