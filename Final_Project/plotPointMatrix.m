function points = plotPointMatrix(image_set, features, point_match_matrix, k)
%PLOTPOINTMATRIX Summary of this function goes here
%   Detailed explanation goes here

idx_image = find(point_match_matrix(:,k)); % nr. of images
N = numel(idx_image);

width = size(image_set{1},2);

% join all equally sized images
image_merge = [];
for i = 1:N
    j = idx_image(i);
    image_merge = [image_merge image_set{j}];
end

% plot merged images
imshow(image_merge);

% make point array
points = zeros(2,N);
points_plot = zeros(2,N);
for i = 1:N
    j = idx_image(i);
    match_idx = point_match_matrix(j,k);
    points(:,i) = features{j}(1:2,match_idx);
    % add image width to x point
    points_plot(:,i) = [points(1,i)+(i-1)*width; points(2,i)];
end

% plot points on merged images
hold on
plot(points_plot(1,:), points_plot(2,:),'g.','MarkerSize',14,'LineWidth',2)
plot(points_plot(1,:), points_plot(2,:),'r')

return
end

