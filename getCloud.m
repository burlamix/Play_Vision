function cloud = getCloud(Image_set, S_stitched, D)
%GETCLOUD Summary of this function goes here
%   Detailed explanation goes here


N = length(D);
points2D = cell(1,N);
imagePoints = cell(1,N);
for i =  1:N
    % pixel location of points in each image
    points2D{i} = round(D{i}(1:2,:),0);
    % assign the color of each point into an RGB array
    for j = 1:size(points2D{i},2)
        % note: rows = y loc., columns = x loc.
        imagePoints{i}(j,:) = double(Image_set{i}(points2D{i}(2,j),points2D{i}(1,j),:))./256;
    end
end

% convert cell array to matrix
color_cloud = cell2mat(imagePoints');
% construct cloud
cloud = pointCloud(S_stitched', 'Color', color_cloud);


end

