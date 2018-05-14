
% Load data
ImL = teddy{1};
ImR = teddy{2};

%% Left - Select n points on left image
n = 3;
% Get coordinates
figure
imshow(ImL)
[v,u] = ginput(n);
close;

% Plot left image and points
figure; 
subplot(121);
imshow(ImL); 
title('Selected points on left image'); 
hold on;
plot(v,u,'g+','MarkerSize',14,'LineWidth',2)

%% Right - Draw epipolar line on right image
% Plot right image 
subplot(122); 
imshow(ImR);
title('Epipolar lines on right image'); 
hold on;

% Find epipolar lines (Point in ImL is line in ImR)
epoLine = F*[v';u';ones(1,n)];
% Scale line to image
PlotPoints = lineToBorderPoints(epoLine',size(ImR));
line(PlotPoints(:,[1,3])',PlotPoints(:,[2,4])','Color','green');
%% Check the constraint
% PlotPoints used for epipolar line on right image
Pr = [PlotPoints(:,1)' PlotPoints(:,1)'; PlotPoints(:,2)' PlotPoints(:,2)'; ones(1,n*2)]';
% Slected points on the left image
Pl = [v' v'; u' u'; ones(1,n*2)];
% Constraint: Pr*F*Pl~=0
Zero_con = trace(Pr*F*Pl);