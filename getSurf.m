function [XYZr, Colorsr, simp_idx, simp_idx_filt] = getSurf(cloud, max_length)
%GETSURF Summary of this function goes here
%   Detailed explanation goes here
% get point locs.
[XYZr,xa,~] = unique(round(cloud.Location,0),'rows');
Colorsr = cloud.Color(xa,:);

% simplex indices for triangulation 
simp_idx = delaunayn(XYZr(:,[2 1]));

% filter out simplices that are long in x-y direction
si = zeros(size(simp_idx,1),3);
for i = 1:size(simp_idx,1)
    % point indices
    p0 = simp_idx(i,1);
    p1 = simp_idx(i,2);
    p2 = simp_idx(i,3);
    % vertex locations in x-y
    v0 = XYZr(p0,1:2);
    v1 = XYZr(p1,1:2);
    v2 = XYZr(p2,1:2);
    % simplex lengths
    si(i,1) = pdist([v0;v1]);
    si(i,2) = pdist([v1;v2]);
    si(i,3) = pdist([v2;v0]);
end

% remove some 'wrong' simplices
simp_idx_filt = simp_idx;
si_max = max(si,[],2);
% estimate a threshold, if not supplied
if nargin < 2
    max_length = round(mean(si_max) + std(si_max),0);
    fprintf('No threshold supplied by user. Using threshold %d', max_length);
end

% simplices to be removed:
[simp_rem,~] = find(si_max>max_length);
simp_idx_filt(simp_rem,:) = [];


end

