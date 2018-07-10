function []  = plotEpilines(image_left, image_right, p_L, p_R, F)
%EPILINES Summary of this function goes here

%% Define Colorlist
c = [1 1 0; 1 0 1; 0 1 1; 1 0 0; 0 1 0; 0 0 1; 1 1 1; 0 0 0];
m = size(c,1); % nr. of colors

%%   Check for Homogeneousity
n = size(p_L,2);

if size(p_L,1) == 2
    p_L(3,:) = ones(1,n); % make pL homogeneous if it isnt 
    p_R(3,:) = ones(1,n); % make pR homogeneous if it isnt 
end



%% Plot left image and points
%figure; 
subplot(2,2,1);
imshow(image_left); 
title('Selected points on left image'); 
hold on;
for i = 1:n
    plot(p_L(1,i),p_L(2,i),'+','Color', c(mod(i,m)+1,:), 'MarkerSize',14,'LineWidth',2)
end

%% Draw epipolar line on right image
% Plot right image 
subplot(2,2,2); 
imshow(image_right);
title('Epipolar lines on right image'); 
hold on;

% Find epipolar lines 
epoLine = F*p_L; %Format: Ax + By + C = 0
% Scale line to image
PlotPoints = lineToBorderPoints(epoLine',size(image_right));
for i = 1:n
    plot(PlotPoints(i,[1,3])',PlotPoints(i,[2,4])','Color',c(mod(i,m)+1,:),'LineWidth',1);
end

%% Draw epipolar line on left image 
subplot(2,2,3);
imshow(image_left);  
title('Epipolar lines on left image'); 
hold on;

% Find epipolar lines 
epoLine = p_R'*F; %Format: Ax + By + C = 0
% Scale line to image
PlotPoints = lineToBorderPoints(epoLine,size(image_left));
for i = 1:n
    plot(PlotPoints(i,[1,3])',PlotPoints(i,[2,4])','Color',c(mod(i,m)+1,:),'LineWidth',1);
end

%% Plot right image and points
subplot(2,2,4);
imshow(image_right); 
title('Selected points on right image'); 
hold on;
for i = 1:n
    plot(p_R(1,i),p_R(2,i),'+','Color', c(mod(i,m)+1,:), 'MarkerSize',14,'LineWidth',2)
end
end
