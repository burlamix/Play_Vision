function[F, idx_inliers, pL, pR] = eight_point_ransac( points_left, points_right, sampson_threshold )
%EIGHT_POINT_RANSAC Summary of this function goes here
%   Detailed explanation goes here

pL = points_left;
pR = points_right;
thr = sampson_threshold;


%% 1.) Normalize points
[pL_N, TL] = normalizePoints(pL);
[pR_N, TR] = normalizePoints(pR);

%% 2.) Apply RANSAC

% sample 8 points and set nr. of loops
m = 8; % minimal seed
w = 0.5; % prob of selecting inlier
L = ceil((log10(1-0.999)/log10(1-w^m))); % nr. of loops
%L = 1000;

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
        % simplify the notation
        pL_j = pL_N(:,j); 
        pR_j = pR_N(:,j); 
        % more simplified notations because matlab can't into indexing
        FpL2 = (F_N*pL_j).^2;
        FTpL2 = (F_N'*pL_j).^2;
        % Compute pairwise Sampson distance between all pixels
        samp_dist(j) = ((pR_j'*F_N*pL_j)^2)...
                        /(FpL2(1)+FpL2(2)+FTpL2(1)+FTpL2(2));
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
    end
end
%%% 2c.) Return best fit
F_N = F_best;
idx_inliers = idx_best;
pL = pL_best;
pR = pR_best;

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
