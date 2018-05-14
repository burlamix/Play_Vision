clear; clc; close(gcf);

%% 0a) Nr. of Images
N = 2;

%% 0b) Pre-allocating images
teddy = cell(1,N);
teddy_gray = cell(1,N);
feats = cell(1,N);
desc = cell(1,N);

%% 1a) Load Images and Perform SIFT
fprintf("Loading images and performing SIFT\n");
for i = 1:N
    tic;
    fname = sprintf('Teddybear/obj02_%03d.jpg',i);
    teddy{1,i} = imread(fname);
    
    teddy_gray{i} = single(rgb2gray(teddy{1,i}))./255;
    [feats{i}, desc{i}] = vl_sift(teddy_gray{i}, 'PeakThresh', 0.04, 'EdgeThresh',10);
    Xfeats = feats{i}(1,:);
    Yfeats = feats{i}(2,:);
    fprintf('finished image %d/%d, %d features found...', i, N, length(Xfeats));        
    subplot(1,2,i)
    hold on
    imshow(teddy{i});
    scatter(Xfeats,Yfeats,'r');
    hold off
    toc;    
end

pause
close(gcf)

%% 1b) Compute matches 
fprintf("Computing matches between images...\n");
matches = cell(1,N-1);
match_coords = cell(N-1,2);
for i = 1:N-1
    [matches{i}, scores{i}] = vl_ubcmatch(desc{i},desc{i+1});
    match_coords{i,1} = feats{i}(1:2,matches{i}(1,:));
    match_coords{i,2} = feats{i+1}(1:2,matches{i}(2,:));
end
fprintf("done\n")

%% 1c) Apply RANSAC
fprintf("Applying RANSAC...")
pix_thr = 10;
for i = 1:N-1
    [X, n_inliers, pix_dist, p_L, p_R] = ransac_func(feats{i}, feats{i+1}, matches{i}, pix_thr); 
end

%% 1d) Plots before/after RANSAC 
plotMatches(teddy{1}, teddy{2}, match_coords{1,1}, match_coords{1,2});
pause
plotMatches(teddy{1}, teddy{2}, p_L, p_R);
pause
close(gcf)

%% 2 & 3) Compute fundamental matrices
% Fundamental matrix without normalization
F = computeFundamental(p_L, p_R);

% Make matched points homogeneous
p_Lh = [p_L; ones(1, size(p_L, 2))];
p_Rh = [p_R; ones(1, size(p_R, 2))];

% Normalize matched points
[p_Ln, TL] = normalizePoints(p_Lh);
[p_Rn, TR] = normalizePoints(p_Rh);

% Normalized fundamental matrix
F_N = computeFundamental(p_Ln, p_Rn);
F_T = TR'*F_N*TL; 

%% 4) Use RANSAC for fundamental matrices

% Make all feature points homogeneous
pf_Lh = [feats{i}(1:2,:); ones(1,size(feats{i},2))];
pf_Rh = [feats{i+1}(1:2,:); ones(1,size(feats{i+1},2))];

% Normalize feature points
[pf_Ln, ~] = normalizePoints(pf_Lh);
[pf_Rn, ~] = normalizePoints(pf_Rh);

samp_thr = 0.005;
% Use RANSAC on fundamental matrix for the normalized feature points
[F_R, inliers_R, samp_dist, p_L_F, p_R_F] = ransac_fundamental(pf_Ln, pf_Rn, matches{i}, samp_thr);

%% 5) Draw epipolar lines
% Normalization
plotEpilines(teddy{1}, teddy{2}, p_Lh(:,1:3), p_Rh(:,1:3), F_T)
pause;
close(gcf)
% Ransac
plotEpilines(teddy{1}, teddy{2}, p_Lh(:,1:3), p_Rh(:,1:3), F_R)