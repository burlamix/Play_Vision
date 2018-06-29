function [match_indices, matched_points_left, matched_points_right] = getMatches(feature_points, descriptors)
%GETMATCHES Summary of this function goes here
%   Detailed explanation goes here
N = length(feature_points);
% pre-allocation of match indices
match_indices = cell(1,N);
% pre-allocation of matched point locations
matched_points_left = cell(1,N);
matched_points_right = cell(1,N);

% get matched points of image pairs (left-right)
for i = 1:N
    if i < N
        % find match indices in both images
        match_indices{i} = vl_ubcmatch(descriptors{i},descriptors{i+1});
        % find matched points coordinates in right image
        matched_points_right{i} = feature_points{i+1}(:,match_indices{i}(2,:));
    else
        % find match indices in both images
        match_indices{i} = vl_ubcmatch(descriptors{i},descriptors{1});
        % find matched points coordinates in right image
        matched_points_right{i} = feature_points{1}(:,match_indices{i}(2,:));
    end
    % find matched points coordinates in left image  
    matched_points_left{i} = feature_points{i}(:,match_indices{i}(1,:));
    fprintf('%d/%d... ', i,N);        
end

end

