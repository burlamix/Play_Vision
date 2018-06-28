function[F, idx_inliers, pL, pR, sampdist] = eight_point_ransac( points_left, points_right, sampson_threshold, n_loops)
%EIGHT_POINT_RANSAC Summary of this function goes here
%   Detailed explanation goes here
pL = points_left;
pR = points_right;
thr = sampson_threshold;


%% 1.) Normalize points
[pL_N, TL] = normalizePoints(pL);
[pR_N, TR] = normalizePoints(pR);

%% 2.) Apply RANSAC
m = 8; % minimal seed
% check if nr. of loops is given, else estimate
if nargin< 4
    % sample 8 points and set nr. of loops
    w = 0.5; % prob of selecting inlier
    L = ceil((log10(1-0.999)/log10(1-w^m))); % nr. of loops
else
    L = n_loops;
end

% intial conditions
N = size(pL, 2); % nr. of points
n_inliers_best = 0;
F_best = zeros(3);
pL_best = zeros(2,1);
pR_best = zeros(2,1);
samp_dist = zeros(1,N); % sampson distances
idx_best = 0;

for i = 1:L 
    %%% 2a) Sampling Stage:
    % m unique samples (from columns)
    s = randperm(N,m);
    % sample normalized point coordinates
    pL_s = pL_N(:,s);
    pR_s = pR_N(:,s);
    % Compute normalized fundamental matrix w/ sampled points
    F_N = computeFundamental(pL_s, pR_s);
    %%% 2b) Concensus Stage:
    for j = 1:N
        % Sampson error
        FpL = F_N*pL_N;
        FtpR = F_N'*pR_N;
        
        % Compute pairwise Sampson distance between all pixels
        samp_dist(j) = ((pR_N(:,j)'*F_N*pL_N(:,j))^2)...
                        /(FpL(1,j)^2+FpL(2,j)^2+FtpR(1,j)^2+FtpR(2,j)^2);
    end
    % Find indices of inlier points
    idx_inliers = find(samp_dist<thr);
    % Compute nr. of inliers
    n_inliers = length(idx_inliers);
    % Update F with most amount of inliers
    if (n_inliers > n_inliers_best)
        idx_best = idx_inliers;
        F_best = F_N;
        n_inliers_best = n_inliers;
        pL_best = pL(:,idx_inliers); % return the unnormalized points
        pR_best = pR(:,idx_inliers); 
        sampdist_best = samp_dist;
    end
end
%%% 2c.) Return best fit
F_N  = F_best;
idx_inliers = idx_best;
pL = pL_best;
pR = pR_best;
sampdist = sampdist_best;
%% 3.) Denormalize F
F = TR'*F_N*TL;
return
end

function [pn, T] = normalizePoints(p)
%NORMALIZEPOINTS Summary of this function goes here
%   p = 2xN or homogeneous 3xN matrix
if size(p,1) == 2
    p(3,:) = ones(1,size(p,2)); % make p homogeneous if it isnt 
end

x = p(1,:);
y = p(2,:);

mx = mean(x);
my = mean(y);

d = 1/size(p,2)*(sum(sqrt((x-mx).^2+(y-my).^2)));

T = [sqrt(2)/d 0 -mx*sqrt(2)/d;...
     0 sqrt(2)/d -my*sqrt(2)/d;...
     0 0 1];

pn = T*p;
end

function F = computeFundamental(p_L, p_R)
%COMPUTEFUNDAMENTAL Summary of this function goes here
%   p_L, p_R = 2xN matrix (or 3xN, doesn't really matter)

x_L = p_L(1,:)'; % transpose necessary for construction of A
y_L = p_L(2,:)';
x_R = p_R(1,:)';
y_R = p_R(2,:)';

A = [x_L.*x_R x_L.*y_R x_L y_L.*x_R y_L.*y_R y_L x_R y_R ones(size(x_L))];

% SVD of A (F = eigenvector corr. to smallest eigenvalue of A'*A)
[~, ~, V] = svd(A);
F=reshape(V(:,9),[3 3]);

% SVD of F
[Uf, Df, Vf] = svd(F);

% Force F to singularity
Df(3,3) = 0;
F = Uf*Df*Vf';
return
end
