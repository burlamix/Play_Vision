function []  = plotEpilines(image_left, image_right, p_L, p_R, F)
%EPILINES Summary of this function goes here
%   use homogeneous 3xN points

N = size(p_L, 2);
%% Plot left image and points
figure; 
subplot(1,2,1);
imshow(image_left); 
title('Selected points on left image'); 
hold on;
plot(p_L(1,:),p_L(2,:),'g+','MarkerSize',14,'LineWidth',2)

%% Draw epipolar line on right image
% Plot right image 
subplot(1,2,2); 
imshow(image_right);
title('Epipolar lines on right image'); 
hold on;

% Find epipolar lines 
epoLine = F*p_L; %Format: Ax + By + C = 0
% Scale line to image
PlotPoints = lineToBorderPoints(epoLine',size(image_right));
line(PlotPoints(:,[1,3])',PlotPoints(:,[2,4])','Color','green');


end
