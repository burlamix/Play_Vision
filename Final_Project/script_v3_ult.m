%% clear;
clear; clc;
close all;

%% Pre-allocating images
N  = 16;
castle = cell(1,N);
castle_gray = cell(1,N);
feats = cell(1,N);
desc = cell(1,N);

%% Load Images and Perform SIFT
%tic;
%figure;
for i = 1:N
    %fname = sprintf('model_castle/8ADT%d.JPG',8585+i); % castle
    fname = sprintf('teddybearPNG/obj02_%03d.png',i); % teddy
    castle{i} = imread(fname);
    %castle{i} = castle{i}(500:2200,500:3500,:);
    castle_gray{i} = single(rgb2gray(castle{1,i}))./255;
    [feats{i}, desc{i}] = vl_sift(castle_gray{i}, 'PeakThresh', 0.005, 'EdgeThresh',10);
    Xfeats = feats{i}(1,:);
    Yfeats = feats{i}(2,:);
    fprintf('finished image %d/%d\n', i,N);        
%     hold off
%     imshow(castle{i});
%     hold on
%     scatter(Xfeats,Yfeats);
    %pause()
%toc;    
end
%close(gcf)

im_height = size(castle{1},1);
im_width = size(castle{1},2);

matches = cell(1,N);
pL = cell(1,N);
pR = cell(1,N);
fprintf('getting matches...');
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
fprintf('done.\n');

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
    [F(:,:,i), idx_inliers{i}, pL_best{i}, pR_best{i}] = eight_point_ransac(pL{i}, pR{i}, 0.00005);
    matches_best{i} = matches{i}(:,idx_inliers{i}); % feature columns of best matches
    n_inliers(i) = length(idx_inliers{i});
    fprintf('loop %d/%d finished.. returned %d/%d inliers.\n', i,N, n_inliers(i), n_points(i));
end
toc;


% plot the epipolar lines of 10 (random) inliers of image pair with least matches
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

% loop from second to last imageset
for i = 2:N
    % find indices of mutual points
    [~, ia, ib] = intersect(point_mat(i,:), matches_pv{i}(1,:));

    % write next row of matrix
    point_mat(i+1,ia) = matches_pv{i}(2,ib);
    % clear all the intersecting matches
    matches_pv{i}(:,ib) = [];
    % add the remaining matches to the matrix
    point_mat = horzcat(point_mat, vertcat(zeros(i-1,size(matches_pv{i},2)), matches_pv{i}));
        
end
% result = N+1 rows: needs to be fixed
% check where row 1 and N+1 intersect
[~, ia, ib] = intersect(point_mat(N+1,:), point_mat(1,:));
% add col.s of ia to ib, remove ia
point_mat(:,ib) = point_mat(:,ia) + point_mat(:,ib);
point_mat(:,ia) = [];
% now add remaining cols of N+1 to row 1
point_mat(1,:) = max(point_mat(1,:),point_mat(N+1,:));
% now sort point matrix by cols. of first row (optional)
point_mat(point_mat==0) = NaN;
point_mat = sortrows(point_mat',1)';
point_mat(isnan(point_mat)) = 0;
% remove row N+1
point_mat(N+1,:) = [];
%%% some duplicate entries remain due to matches
fprintf("done\n")


%% Plot one random column of the matrix to test it
k = randi(size(point_mat,2));
figure;
pts_SFM = plotPointMatrix(castle, feats, point_mat, k);

%% 3.5) Affine Stitching
% pix_thr = 1;
% T = zeros(3,3,N);
% T(:,:,1) = eye(3);
% 
% for i = 1:(N-1)
%     [~,~,~,~,T(:,:,i+1)] = ransac_affine(pL{i}, pR{i}, pix_thr);
% end

%tmp{1} = castle{1}; tmp{2} = castle{2}; tmp{3} = castle{3};
%figure;
%stitchImages(tmp, T);

%% 4+5.) Stitching and Structure-from-Motion
% nr. of images to compare matches between ( > 2)
m = 4;
% build image comparison matrix
sets = [1:N]';
for i = 2:m
    sets(:,i) = sets(:,i-1) + 1;
    for j= 1:N
        if sets(j,i) > N
            sets(j,i) = sets(j,i) - N;
        end
    end
end

set_mat = cell(1,N);
points2D = cell(1,N);
imagePoints = cell(1,N);
p_im = cell(m-1,N);
% big-ass for loop
for i = 1:N
    %%% big image set
    set = sets(i,:);
    % find indices of current big set witn non-zero entries
    idx_sets = all(point_mat(set,:));
    % match values of sets in point matrix
    pm_sets = point_mat(set, idx_sets);
    
    %%% smaller image sets
    set1 = set(1:(m-1));
    set2 = set(2:m);
    % find indices of current small sets witn non-zero entries
    idx_set1 = all(point_mat(set1,:));
    idx_set2 = all(point_mat(set2,:));
    % match values of small sets in point matrix
    pm_set1 = point_mat(set1, idx_set1);
    pm_set2 = point_mat(set2, idx_set2);
    
    fprintf('sets created...');
    %%% generate list of indexes where set 1 and set 2 are part of sets
    set_mat{i} = zeros(2,size(pm_sets,2));
    for j = 1:size(pm_sets,2)
        set_mat{i}(1,j) = max(find(pm_set1(1:(m-1),:) == pm_sets(1:(m-1),j)))/(m-1);
        set_mat{i}(2,j) = max(find(pm_set2(1:(m-1),:) == pm_sets(2:m,j)))/(m-1);
    end
    fprintf('set matrix generated....');
    % get image points
    D = [];
    for j = 1:(m-1)
        D(2*j-1:2*j,:) = feats{set1(j)}(1:2,pm_set1(j,:));
    end
    
    fprintf('D generated..')
    %%% structure from motion
    % Center D
    numPts = size(D,2);
    Dn{i} = D - repmat(sum(D,2)/numPts, 1, numPts);
    % SVD of normalized D
    [U, W, V] = svd(Dn{i});
    % Force to rank 3
    U3 = U(:,1:3);
    V3 = V(:,1:3);
    W3 = W(1:3,1:3);
    % Recreate structure and motion
    M = U3*sqrt(W3);
    S = sqrt(W3)*V3';
    
    %%%solve for affine ambiguity (copypaste)
    A1 = M(1:2,:);
    L0=pinv(A1'*A1);
    save('M','M')
    lsq_opts = optimoptions('lsqnonlin','Display','off'); % hide msg in prompt
    [L, res(i)] = lsqnonlin(@myfun,L0,[],[],lsq_opts);
    fprintf('Finished LSQ...');
    %L = nearestSPD(L);
    [C,p(i)] = chol(L,'lower');
    fprintf('Finished chol, p = %d...',p(i));
    Mn{i} = M*C;
    Sn{i} = pinv(C)*S;
    fprintf('New M and S...');
    % for pointcloud
    points2D{i} = round(D(1:2,:),0);
    for j = 1:size(points2D{i},2)
        imagePoints{i}(j,:) = double(castle{i}(points2D{i}(2,j),points2D{i}(1,j),:))./256;
    end
    
    fprintf('Finished 3D point set %d/%d\n', i, N);
    %%% plot stuff
%     subplot(4,4,i);
%     plotMatches3Im(castle{set1(1)}, castle{set1(2)}, castle{set1(3)}, p_im1, p_im2, p_im3)
end

%     %%% get image points (clean)
%     p_im1 = feats{ss(1)}(1:2,pm_ss(1,Cmat(i,:)));
%     p_im2 = feats{ss(2)}(1:2,pm_ss(2,Cmat(i,:)));
%     p_im3 = feats{ss(3)}(1:2,pm_ss(3,Cmat(i,:)));
    

%% Procrustes
Sp = Sn;

X = cell(1,m-1);
Y = cell(1,m-1);
Z = cell(1,m-1);
transform = cell(1,m-1);
figure;
% 3d transform
for i = 1:(N-1)
    % select points
    X{i} = Sn{i}(:,set_mat{i}(1,:))'; 
    Y{i} = Sn{i+1}(:,set_mat{i}(2,:))';
    % transform matching points across big set to prev. viewpoints
    [d,Z{i},transform{i}]=procrustes(X{i},Y{i});
    % transform all points to prev. viewpoint
    Sn{i+1} = (transform{i}.b.*Sn{i+1}'*transform{i}.T + transform{i}.c(1,:).*ones(length(Sn{i+1}),3))';
%     % plot
%     % all points
%     plot3(Sn{i}(1,:),Sn{i}(2,:),Sn{i}(3,:),'kx',Sp{i+1}(1,:),Sp{i+1}(2,:),Sp{i+1}(3,:),'c.',Sn{i+1}(1,:),Sn{i+1}(2,:),Sn{i+1}(3,:),'cx', 'MarkerSize', 5);
%     hold on
%     % points used for 3D transform
%     plot3(X{i}(:,1),X{i}(:,2),X{i}(:,3),'rx',Y{i}(:,1),Y{i}(:,2),Y{i}(:,3),'b.',Z{i}(:,1),Z{i}(:,2),Z{i}(:,3),'bx', 'MarkerSize', 10);
%     hold off
%     title(sprintf('plot %d: images %d, %d, %d, %d', i, sets(i,1),sets(i,2),sets(i,3),sets(i,4)))
%     pause;
end

%% get pointcloud

Sn_cloud = cell2mat(Sn);
color_cloud = cell2mat(imagePoints');

cloud = pointCloud(Sn_cloud', 'Color', color_cloud);

% plot pointcloud
figure;
pcshow(cloud,'MarkerSize', 50)

%% interpolate the pointcloud with surf...
% get point locs.
[XYZr,xa,~] = unique(round(cloud.Location,0),'rows');
Colorsr = cloud.Color(xa,:);

% ordinary surface plot w/ colors
Z_mat = [];
for i=1:size(XYZr,1)
    Z_mat(500+XYZr(i,1),500+XYZr(i,2)) = XYZr(i,3) + 500;
    C_mat(500+XYZr(i,1),500+XYZr(i,2),:) = Colorsr(i,:);
end
XX = 1:size(Z_mat,1);
YY = 1:size(Z_mat,2);
figure;
surf(Z_mat,C_mat,'EdgeColor','interp');

% simplices for triangulation 
TRIeval = delaunayn(XYZr(:,[2 1]));

% triplot w/ colors
figure;
trisurf(TRIeval,XYZr(:,1),XYZr(:,2),XYZr(:,3),[1:size(XYZr,1)]','FaceColor','flat','EdgeColor','interp')
colormap(double(Colorsr)./256);

%% filter out simplices that are long in x/y direction AND have a large area
ang =zeros(size(TRIeval,1),3);
area = zeros(size(TRIeval,1),1);
for i = 1:size(TRIeval,1)
    % point indices
    p0 = TRIeval(i,1);
    p1 = TRIeval(i,2);
    p2 = TRIeval(i,3);
    
    % vertex locations in x-y
    v0 = XYZr(p0,1:2);
    v1 = XYZr(p1,1:2);
    v2 = XYZr(p2,1:2);
    
    % get angles btwn points in deg
    ang(i,1) = atan2d(abs(det([v2-v0;v1-v0])),dot(v2-v0,v1-v0));
    ang(i,2) = atan2d(abs(det([v0-v1;v2-v1])),dot(v0-v1,v2-v1));
    ang(i,3) = atan2d(abs(det([v1-v2;v0-v2])),dot(v1-v2,v0-v2));
    
    % get area
    area(i,1) = polyarea([v0(1) v1(1) v2(1)],[v0(2) v1(2) v2(2)]);
    %plot3([s01(1) s12(1) s20(1) s01(1)],[s01(2) s12(2) s20(2) s01(2)],[s01(3) s12(3) s20(3) s01(3)])
    %if (min(ang) < 5 &  
end

% while...
TRInew = TRIeval;
for i = 1:size(TRIeval,1)
    if (min(ang(i,:)) < 20 && area(i) > (mean(area)+std(area)))
        TRInew(i,:) = [];
    end
end

% triplot w/ colors
figure;
trisurf(TRInew,XYZr(:,1),XYZr(:,2),XYZr(:,3),[1:size(XYZr,1)]','FaceColor','flat','EdgeColor','interp')
colormap(double(Colorsr)./256);
    