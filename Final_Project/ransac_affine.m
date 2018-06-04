function [inliers, pix_dist, pL, pR, T] = ransac_affine(points_left, points_right, pix_thr)
%RANSAC_CASTLE Summary of this function goes here
%   Detailed explanation goes here

m = 3; % minimal seed
w = 0.3; % prob of selecting inlier
L = ceil((log10(1-0.9999)/log10(1-w^m))); % nr. of loops
inliers_best = 0;



% define point arrays
xL = points_left(1,:);
yL = points_left(2,:);
xR = points_right(1,:);
yR = points_right(2,:);

N = length(xL);

% pre-allocate transformation matrices
A = zeros(2*m,6);
b = zeros(2*m,1);

% Start Loop
for j = 1:L
    %%% Sampling Stage
    % m unique samples (from columns)
    idx = randperm(N,m);
    % sample point coordinates
    xL_s = xL(idx);
    yL_s = yL(idx);
    xR_s = xR(idx);
    yR_s = yR(idx);
    % start loop to build system of equations
    for i = 1:m 
        % construct A matrix from points of left image
        A((2*i-1):(2*i),:) = [xR_s(i) yR_s(i) 0 0 1 0;...
                              0 0 xR_s(i) yR_s(i) 0 1];
        % construct b matrix from points of right image
        b((2*i-1):(2*i)) = [xL_s(i);...
                            yL_s(i)];
    end
    % Obtain affine transformation X
    X = pinv(A)*b; % m1;m2;m3;m4;t1;t2
    m1 = X(1); m2 = X(2); m3 = X(3); m4 = X(4); t1 = X(5); t2 = X(6);
    %%% Concensus stage
    % Estimate pixel location on right image
    xL_e = xR.*m1+yR.*m2+t1;
    yL_e = xR.*m3+yR.*m4+t2;
    % Compute pairwise L2 distance between all pixels
    pix_dist = sqrt(abs(xL_e-xL).^2+abs(yL_e-yL).^2);
    % Compute indices of inlier points
    pix_inliers = find(pix_dist<pix_thr);
    % Compute nr. of inliers
    inliers = length(pix_inliers);
    % Update X with most amount of inliers
    if (inliers > inliers_best)
        inliers_best = inliers;
        X_best = X;
        pix_dist_best = pix_dist;
        xL_best = xL(pix_inliers); 
        yL_best = yL(pix_inliers); 
        xR_best = xR(pix_inliers); 
        yR_best = yR(pix_inliers);
    end
  
end
% Define X as the best found fit
X = X_best;
m1 = X(1); m2 = X(2); m3 = X(3); m4 = X(4); t1 = X(5); t2 = X(6);
% Matlab version of T
T = [m1 m3 0;...
    m2 m4 0;...
    t1 t2 1];

inliers = inliers_best;
pix_dist = pix_dist_best;
% Return coordinates of points of best fit
pL = [xL_best; yL_best];
pR = [xR_best; yR_best];
fprintf("\nBest fit returned %d inliers out of %d matches after %d loops\n", inliers, N, L);
return
end

