function [ X, xL_best, yL_best, xR_best, yR_best] = ransac_castle(feats_prev, feats_curr, matches, pix_thr)
%RANSAC_CASTLE Summary of this function goes here
%   Detailed explanation goes here

m = 50; % nr. of sample pairs
v = 0.15; % prob of selecting outlier
N = ceil((log10(1-0.99)/log10(1-(1-v)^m))); % nr. of loops
inliers_best = 0;


for j = 1:N
    p = datasample(matches,m,2,'Replace',false); %second axis, unique samples
    A = []; b = [];
    for i = 1:m % compute A matrix
        x_L(i) = feats_prev(1,p(1,i));
        y_L(i) = feats_prev(2,p(1,i));
        x_R(i) = feats_curr(1,p(2,i));
        y_R(i) = feats_curr(2,p(2,i));
        A = [A;...
            x_L(i) y_L(i) 0 0 1 0;...
            0 0 x_L(i) y_L(i) 0 1];
        b = [b;...
            x_R(i);...
            y_R(i)];
    end
    X = pinv(A)*b; % m1;m2;m3;m4;t1;t2
    m1 = X(1); m2 = X(2); m3 = X(3); m4 = X(4); t1 = X(5); t2 = X(6);
    % Estimated pixels on right image
    X_R = x_L.*m1+y_L.*m2+t1;
    Y_R = x_L.*m3+y_L.*m4+t2;
    pix_dist = sqrt(abs(x_R-X_R).^2+abs(y_R-Y_R).^2); % pairwise L2 dist
    inliers = length(find(pix_dist<pix_thr)); % nr. of inliers
    if (inliers > inliers_best)
        inliers_best = inliers;
        X_best = X;
        xL_best = x_L; yL_best = y_L; xR_best = x_R; yR_best = y_R;
    end
    
    if (inliers_best == m)
        break
    end
end
X = X_best;
return
end

