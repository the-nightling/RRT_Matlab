function d = euclidianDistSquare(xs,X)
% compute the squares of the euclidian distance from xs to every point in X
	Y = repmat(xs,1,size(X,2));
	d = sum((X-Y).^2,1);
end
