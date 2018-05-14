function [pn, T] = normalizePoints(p)
%NORMALIZEPOINTS Summary of this function goes here
%   p = homogeneous 3xN matrix
x = p(1,:);
y = p(2,:);

mx = mean(x);
my = mean(y);

d = 1/size(p,2)*(sum(sqrt((x-mx).^2+(y-my).^2)));

T = [sqrt(2)/d 0 -mx*sqrt(2)/d;...
     0 sqrt(2)/d -my*sqrt(2)/d;...
     0 0 1];

pn = T*p;
end

