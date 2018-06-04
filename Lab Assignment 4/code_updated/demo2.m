%function demo3(tracked)
%A demo of the structure from motion that shows the 3d structure of the
%points in the space. Either from the original points or the tracked points
%INPUT
%- tracked: boolean that states whether to use the tracked points
%           if set to true the tracked points will be used otherwise
%           the ground truth is used. (default = false)
%
%OUTPUT
% an image showing the points in their estimated 3-dimensional position,
% where the yellow dots are the ground truth and the pink dots the tracked
% points from the LKtracker
%- M: The transformation matrix size of 2nx3. Where n is the number of 
%     cameras i.e. images.
%- S: The estimated 3-dimensional locations of the points (3x#points)
function [M,S] = demo3
close all


% % %load points
Points = textread('model house\measurement_matrix.txt');
% % %Shift the mean of the points to zero using "repmat" command


% % %singular value decomposition
[U,W,V] = svd(Points);

U = 
W = 
V = 

M = 
S = 

save('M','M')

% % %solve for affine ambiguity
A = 
L0=

% Solve for L
L = lsqnonlin(@myfun,L0);
% Recover C
C = chol(L,'lower');
% Update M and S
M = M*C;
S = pinv(C)*S;

plot3(S(1,:),S(2,:),S(3,:),'.y');

%% For the tracked points with LKtracker
% % Repeat the same procedure 

Points = zeros(size(Points));

load('Xpoints')
load('Ypoints')

Points(1:2:end,:)=pointsx;
Points(2:2:end,:)=pointsy;


%Shift the mean of the points to zero

%singular value decomposition
[U,W,V] = svd(Points);

U = 
W = 
V = 

M = 
S = 

%solve for affine ambiguity using non-linear least squares
A = 
L0=
save('M','M')

L = lsqnonlin(@myfun,L0);

C = chol(L,'lower');
M = 
S = 

hold on
plot3(S(1,:),S(2,:),S(3,:),'.m');

end
