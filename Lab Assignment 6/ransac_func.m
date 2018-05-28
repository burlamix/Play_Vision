function [ X, pix_inliers, pix_dist, p_L, p_R] = ransac_func(feats_left, feats_right, matches, pix_thr)
%RANSAC_CASTLE Summary of this function goes here
%   Detailed explanation goes here

m = 8; % minimal seed
w = 0.5; % prob of selecting inlier
L = ceil((log10(1-0.9999)/log10(1-w^m))); % nr. of loops
pc = 0;
inliers_best = 0;

% define point arrays
x_L = feats_left(1,matches(1,:));
y_L = feats_left(2,matches(1,:));
x_R = feats_right(1,matches(2,:));
y_R = feats_right(2,matches(2,:));
% pre-allocate transformation matrices
A = zeros(2*m,6);
b = zeros(2*m,1);

% Start Loop
for j = 1:L
    %%% Sampling Stage
    % m unique samples (from columns)
    [~,idx] = datasample(matches,m,2,'Replace',false);
    % sample point coordinates
    x_L_s = x_L(idx);
    y_L_s = y_L(idx);
    x_R_s = x_R(idx);
    y_R_s = y_R(idx);
    % start loop to build system of equations
    for i = 1:m 
        % construct A matrix from points of left image
        A((2*i-1):(2*i),:) = [x_L_s(i) y_L_s(i) 0 0 1 0;...
                              0 0 x_L_s(i) y_L_s(i) 0 1];
        % construct b matrix from points of right image
        b((2*i-1):(2*i)) = [x_R_s(i);...
                            y_R_s(i)];
    end
    % Obtain affine transformation X
    X = pinv(A)*b; % m1;m2;m3;m4;t1;t2
    m1 = X(1); m2 = X(2); m3 = X(3); m4 = X(4); t1 = X(5); t2 = X(6);
    %%% Concensus stage
    % Estimate pixel location on right image
    X_R = x_L.*m1+y_L.*m2+t1;
    Y_R = x_L.*m3+y_L.*m4+t2;
    % Compute pairwise L2 distance between all pixels
    pix_dist = sqrt(abs(x_R-X_R).^2+abs(y_R-Y_R).^2);
    % Compute indices of inlier points
    pix_inliers = find(pix_dist<pix_thr);
    % Compute nr. of inliers
    inliers = length(pix_inliers);
    % Update X with most amount of inliers
    if (inliers > inliers_best)
        pix_inliers_best = pix_inliers;
        inliers_best = inliers;
        X_best = X;
        pix_dist_best = pix_dist;
        xL_best = x_L(pix_inliers); 
        yL_best = y_L(pix_inliers); 
        xR_best = x_R(pix_inliers); 
        yR_best = y_R(pix_inliers);
    end
    
    % Some timing estimates
    if mod(j,ceil(L/10)) == 0
        pc = pc+10;
        fprintf("%d%% done...", pc);
    end
    
end
% Define X as the best found fit
X = X_best;
inliers = inliers_best;
pix_inliers = pix_inliers_best;
pix_dist = pix_dist_best;
% Return coordinates of points of best fit
p_L = [xL_best; yL_best];
p_R = [xR_best; yR_best];
fprintf("\nBest fit returned %d inliers out of %d matches after %d loops\n", inliers, length(matches), L);
return
end

