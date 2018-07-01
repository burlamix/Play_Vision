function pointviewmatrix = chaining(match_indices)
%CHAINING Summary of this function goes here
%   Detailed explanation goes here

% Nr. of images
N = size(match_indices,2);
pointviewmatrix = [];
% first two rows of the point-view matrix
pointviewmatrix(1:2,:) = match_indices{1}(1:2,:);

% loop from second to last imageset
for i = 2:N
    % find indices of mutual matches
    [~, ia, ib] = intersect(pointviewmatrix(i,:), match_indices{i}(1,:));
    % write next row of matrix
    pointviewmatrix(i+1,ia) = match_indices{i}(2,ib);
    % clear all the intersecting matches
    match_indices{i}(:,ib) = [];
    % add the remaining matches to the matrix
    pointviewmatrix = horzcat(pointviewmatrix, vertcat(zeros(i-1,size(match_indices{i},2)), match_indices{i}));     
end
% check where row 1 and N+1 intersect
[~, ia, ib] = intersect(pointviewmatrix(N+1,:), pointviewmatrix(1,:));
% add col.s of ia to ib, remove ia
pointviewmatrix(:,ib) = pointviewmatrix(:,ia) + pointviewmatrix(:,ib);
pointviewmatrix(:,ia) = [];
% now add remaining cols of N+1 to row 1
pointviewmatrix(1,:) = max(pointviewmatrix(1,:),pointviewmatrix(N+1,:));
% remove row N+1
pointviewmatrix(N+1,:) = [];
% now sort point matrix by cols. of first row (optional)
pointviewmatrix(pointviewmatrix<1) = NaN;
pointviewmatrix = sortrows(pointviewmatrix',1)';
pointviewmatrix(isnan(pointviewmatrix)) = 0;

return
end

