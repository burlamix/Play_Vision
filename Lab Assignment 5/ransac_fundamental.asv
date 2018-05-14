function [F, inliers, samp_dist, p_L, p_R] = ransac_fundamental(points_left, points_right, matches, pix_thr)
%RANSAC_FUNDAMENTAL Summary of this function goes here
%   Detailed explanation goes here
N = size(matches,2);

m = 8; % minimal seed
w = 0.5; % prob of selecting inlier
L = ceil((log10(1-0.9999)/log10(1-w^m))); % nr. of loops
pc = 0;
inliers_best = 0;

% define point arrays
p_L = points_left(:,matches(1,:));
p_R = points_right(:,matches(2,:));


% Start Loop
for j = 1:L
    %%% Sampling Stage
    % m unique samples (from columns)
    [~,idx] = datasample(matches,m,2,'Replace',false);
    % sample point coordinates
    p_L_s = p_L(:,idx);
    p_R_s = p_R(:,idx);
    % Compute fundamental matrix of the 8 sample points
    F = computeFundamental(p_L_s, p_R_s);
    %%% Concensus stage
    samp_dist = zeros(1,N);
    for i = 1:N
        % simplifies notation a bit
        p_Li = p_L(:,i); 
        p_Ri = p_R(:,i);
        % more simplified notations because matlab can't into indexing
        FpL2 = (F*p_Li).^2;
        FTpR2 = (F'*p_Ri).^2;
        % Compute pairwise Sampson distance between all pixels
        samp_dist(i) = ((p_Ri'*F*p_Li)^2)...
                        /(FpL2(1)+FpL2(2)+FTpR2(1)+FTpR2(2));
    end
    % Find indices of inlier points
    pix_inliers = find(samp_dist<pix_thr);
    % Compute nr. of inliers
    inliers = length(pix_inliers);
    % Update F with most amount of inliers
    if (inliers > inliers_best)
        F_best = F;
        inliers_best = inliers;
        pix_dist_best = samp_dist;
        pL_best = p_L(pix_inliers); 
        pR_best = p_R(pix_inliers); 
    end
    
    % Some timing estimates
    if mod(j,ceil(L/10)) == 0
        pc = pc+10;
        fprintf("%d%% done...", pc);
    end
    
end
% Define F as the best found fit
F = F_best;
inliers = inliers_best;
samp_dist = pix_dist_best;
% Return coordinates of points of best fit
p_L = pL_best;
p_R = pR_best;
fprintf("\nBest fit returned %d inliers out of %d matches after %d loops\n", inliers, length(matches), L);
return
end

