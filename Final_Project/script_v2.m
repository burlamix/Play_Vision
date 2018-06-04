%% clear;
clear; clc;
close all;

%% Pre-allocating images
N  = 19;
castle = cell(1,N);
castle_gray = cell(1,N);
feats = cell(1,N);
desc = cell(1,N);

%% Load Images and Perform SIFT
tic;
for i = 1:N
    fname = sprintf('model_castle/8ADT%d.JPG',8585+i);
    castle{i} = imread(fname);
    castle{i} = castle{i}(500:2200,500:3500,:);
    castle_gray{i} = single(rgb2gray(castle{1,i}))./255;
    [feats{i}, desc{i}] = vl_sift(castle_gray{i}, 'PeakThresh', 0.01, 'EdgeThresh',10);
    Xfeats = feats{i}(1,:);
    Yfeats = feats{i}(2,:);
    fprintf('finished image %d/19\n', i);        
%     hold off
%     imshow(castle{i});
%     hold on
%     scatter(Xfeats,Yfeats);
    %pause()
toc;    
end
%close(gcf)

im_height = size(castle{1},1);
im_width = size(castle{1},2);

matches = cell(1,N);
pL = cell(1,N);
pR = cell(1,N);

for i = 1:N
    if i < N
        matches{i} = vl_ubcmatch(desc{i},desc{i+1});
        pR{i} = feats{i+1}(1:2,matches{i}(2,:));
    else
        matches{i} = vl_ubcmatch(desc{i},desc{1});
        pR{i} = feats{1}(1:2,matches{i}(2,:));
    end
    pL{i} = feats{i}(1:2,matches{i}(1,:));
end
toc;

% %% Load images, feature points, descriptors, matches
% tic
% fprintf("reading input...");
% load Images
% load SIFTmatches
% fprintf("done\n");
% toc

%% 2.) Normalized eight-point RANSAC
% Pre-allocate cells
pL_best = cell(1,N);
pR_best = cell(1,N);
n_inliers = zeros(1,N);
n_points = zeros(1,N);
F = zeros(3,3,N);
% Loop over all images
tic;
for i = 1:N
    n_points(i) = size(pL{i},2);
    [F(:,:,i), idx_inliers{i}, pL_best{i}, pR_best{i}] = eight_point_ransac(pL{i}, pR{i}, 0.0005);
    matches_best{i} = matches{i}(:,idx_inliers{i}); % feature columns of best matches
    n_inliers(i) = length(idx_inliers{i});
    fprintf('loop %d/%d finished.. returned %d/%d inliers.\n', i,N, n_inliers(i), n_points(i));
end
toc;


%% plot the epipolar lines of 10 (random) inliers on a (random) image
[~,k] = min(n_inliers);
m = randperm(n_inliers(k),10); % pick 10 random unique samples
figure;
plotEpilines(castle{k}, castle{k+1}, pL_best{k}(:,m), pR_best{k}(:,m), F(:,:,k))

%% 3) Chaining: create point view matrix using BEST matches

% duplicate the match cell
matches_pv = matches_best;

point_mat = [];

% first two rows of the point-view matrix
point_mat(1:2,:) = matches_pv{1}(1:2,1:end);

offset = 0;
% loop from second to second-to-last imageset
for i = 2:N-1
    % find indices of mutual points
    [~, ia, ib] = intersect(matches_pv{i-1}(2,:), matches_pv{i}(1,:));
    % write next row of matrix
    point_mat(i+1,offset+ia) = matches_pv{i}(2,ib);
    % update size of pv matrix with the additional rows
    offset = size(point_mat,2);
    % clear all the intersecting matches
    matches_pv{i}(:,ib) = [];
    % add the remaining matches to the matrix
    point_mat = horzcat(point_mat, vertcat(zeros(i-1,size(matches_pv{i},2)), matches_pv{i}));
end

% now add intersecting matches of final imageset (N,1)
i = N; offset = 0;
[~, ia, ib] = intersect(matches_pv{i-1}(2,:), matches_pv{i}(1,:));
point_mat(i,offset+ia) = matches_pv{i}(2,ib);

fprintf("done\n")


%% 3.5) Affine Stitching
pix_thr = 1;
T = zeros(3,3,N);
T(:,:,1) = eye(3);

for i = 1:(N-1)
    [~,~,~,~,T(:,:,i+1)] = ransac_affine(pL{i}, pR{i}, pix_thr);
end

tmp{1} = castle{1}; tmp{2} = castle{2}; tmp{3} = castle{3};
figure;
stitchImages(tmp, T);

%% 4.) Stitching and Structure-from-Motion
set = [1 2 3];% 2 3 4; 3 4 5; 4 5 6; 5 6 7];%; 7 8 9; 9 10 11; 11 12 13; 13 14 15];%; 15 16 17; 17 18 19; 19 1 2];
n = size(set,1);
D = cell(1,n);
Dn = cell(1,n);
M = cell(1,n);
S = cell(1,n);
numPts = zeros(1,n);
% loop over some image sets
%for i = 1:n
%%% get matches of first 3 images    
% current image set
% s3 = set(i,:); 
% s2 = s3(1:2);

s1 = [1 2];
% find indices of row 1 and 2 witn non-zero entries
idx1 = all(point_mat(s1,:));
% parts of point matrix with 2 consecutive rows of only non-zero entrys
pm1 = point_mat(s1,idx1);
% points of match indices 
p_im1 = feats{s1(1)}(1:2,pm1(1,:));
p_im2 = feats{s1(2)}(1:2,pm1(2,:));




s2 = [2 3];
% find indices of row 2 and 3 witn non-zero entries
idx2 = all(point_mat(s2,:));
% parts of point matrix with 2 consecutive rows of only non-zero entrys
pm2 = point_mat(s2,idx2);
% points of match indices 
p_im2_2 = feats{s2(1)}(1:2,pm2(1,:));
p_im3 = feats{s2(2)}(1:2,pm2(2,:));

s3 = [1 2 3];
% find indices of row 1 2 and 3 witn non-zero entries
idx3 = all(point_mat(s3,:));
% parts of point matrix with 3 consecutive rows of only non-zero entrys
p_im1_all = p_im1(:,idx3);
p_im2_all = feats{s2(1)}(1:2,pm2(1,:));
p_im3_all = feats{s2(2)}(1:2,pm2(2,:));

s2 = [2 3];

%pm3 = point_mat(s3,idx3);
% points in image 1, 2  of set
p_im1 = feats{s2(1)}(1:2,pm2(1,:));
p_im2 = feats{s2(2)}(1:2,pm2(2,:));
%p_im3 = feats{s(3)}(1:2,pm3(3,:));

%%% structure from motion
D{i} = [p_im1; p_im2];
% Center D
numPts(i) = size(D{i},2);
Dn{i} = D{i} - repmat( sum(D{i},2)/numPts(i), 1, numPts(i));
% SVD of normalized D
[U, W, V] = svd(Dn{i});
% Force to rank 3
U3 = U(:,1:3);
V3 = V(:,1:3);
W3 = W(1:3,1:3);
% Recreate structure and motion
M{i} = U3*sqrt(W3);
S{i} = sqrt(W3)*V3';

fprintf('Finished 3D point set %d/%d\n', i, n)
%end

%% test procrustes
n=2;
for i = 1:(n-1)
    pL3D{i} = S{i}(:,idx3{i});
    pR3D{i} = S{i+1}(:,idx3{i});
end
X = pL3D{1}'; Y = pR3D{1}';
[d,Z,transform]=procrustes(X,Y);
Z-X
figure;
plot3(X(:,1),X(:,2),X(:,3),'rx',Y(:,1),Y(:,2),Y(:,3),'b.',Z(:,1),Z(:,2),Z(:,3),'bx');
%% plot results of a random set
%k = randi(n);
k = 1;
figure;
subplot(1,2,1);
plot3(S{k}(1,:),S{k}(2,:),S{k}(3,:),'k.')
subplot(1,2,2);
plot3(M{k}(:,1),M{k}(:,2),M{k}(:,3),'k.')
k = 2;
figure;
subplot(1,2,1);
plot3(S{k}(1,:),S{k}(2,:),S{k}(3,:),'k.')
subplot(1,2,2);
plot3(M{k}(:,1),M{k}(:,2),M{k}(:,3),'k.')
l = randperm(numPts(i),5);
%figure;
%plotMatches3Im(castle{set(k,1)}, castle{set(k,2)}, castle{set(k,3)}, D{k}(1:2,:), D{k}(3:4,:), D{k}(5:6,:))


% %% test procrustes
% pxl = D{1}(1,:);
% pyl = D{1}(2,:);
% pxr = D{1}(3,:);
% pyr = D{1}(4,:);
% [d,Z,transform] = procrustes([pxl' pyl'],[pxr' pyr']);
% %  plot(pxl, pyl,'rx',pxr, pyr,'b.',Z(:,1),Z(:,2),'bx');
% Tmat = horzcat(vertcat(transform.T, transform.c(1,:)), [0; 0; 1]);





%% 5) Remove affine ambiguity
