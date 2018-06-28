%% clear;
clear; clc;
close all;



%% Load Images 

%image_files = dir('model_castle/*.JPG');
image_files = dir('TeddyBearPNG/*.png');
N = length(image_files);

% Pre-allocating images
castle = cell(1,N);
castle_gray = cell(1,N);

fprintf('loading images...');
%figure;
for i = 1:N
    %fname = sprintf('model_castle/8ADT%d.JPG',8585+i); % castle
    %fname = sprintf('teddybearPNG/obj02_%03d.png',i); % teddy
    castle{i} = imread([image_files(i).folder '/' image_files(i).name]);
    %castle{i} = castle{i}(500:2200,500:3500,:); % RoI
    castle_gray{i} = single(rgb2gray(castle{1,i}))./255;    
%     hold off
%     imshow(castle{i});
%     hold on
%     scatter(Xfeats,Yfeats);
    %pause()
end
%close(gcf)
fprintf('done.\n');
im_height = size(castle{1},1);
im_width = size(castle{1},2);

%% Find and combine features
% affine harris/hessian locations
%aff_harris_files = dir('model_castle/*.png.haraff.sift');
%aff_hessian_files = dir('model_castle/*.png.hesaff.sift');
% affine harris/hessian locations
aff_harris_files = dir('TeddyBearPNG/*.png.haraff.sift');
aff_hessian_files = dir('TeddyBearPNG/*.png.hesaff.sift');

% pre-allocation of cells
feat_SIFT = cell(1,N);
descr_SIFT = cell(1,N);
descr_HARRIS = cell(1,N);
descr_HESSIAN = cell(1,N);
descriptors = cell(1,N);

points_SIFT = cell(1,N);
points_HARRIS = cell(1,N);
points_HESSIAN = cell(1,N);
all_feature_points = cell(1,N);

% feature extraction w/ multiple methods
fprintf('extracting features: finished image ');
for i = 1:N
    % Obtain HARRIS corner points
    
    % SIFT feature, descriptor extraction of HARRIS corners... todo
    [feat_SIFT{i}, descr_SIFT{i}] = vl_sift(castle_gray{i}, 'PeakThresh', 0.03, 'EdgeThresh', 20);
    [feat_SIFT{i}, descr_SIFT{i}] = my_vl_sift(castle_gray{i},0.001);
    % AFFINE HARRIS descriptor
    matrix_harris = dlmread([aff_harris_files(i).folder '/' aff_harris_files(i).name], ' ', 2, 0);
    descr_HARRIS{i} = matrix_harris(:,6:end)';
    % AFFINE HESSIAN descriptor
    matrix_hessian = dlmread([aff_hessian_files(i).folder '/' aff_hessian_files(i).name], ' ', 2, 0);
    descr_HESSIAN{i} = matrix_hessian(:,6:end)';
    % Combine descriptors
    descriptors{i} = [descr_SIFT{i} descr_HARRIS{i} descr_HESSIAN{i}];
    
    % Get feature locations
    points_SIFT{i} = feat_SIFT{i}(1:2,:);
    points_HARRIS{i} = matrix_harris(:,1:2)';
    points_HESSIAN{i} = matrix_hessian(:,1:2)';
    % combine all feature points in 2xN format
    all_feature_points{i} = [points_SIFT{i} points_HARRIS{i} points_HESSIAN{i}];
    fprintf('%d/%d... ', i,N);    
end
fprintf('\nfinished feature extraction\n');
%%
i = 1;
figure;
subplot(2,3,1);
imshow(castle{i});
hold on;
plot(points_SIFT{i}(1,:),points_SIFT{i}(2,:),'ko','MarkerSize',12);
subplot(2,3,2);
imshow(castle{i});
hold on;
plot(points_HARRIS{i}(1,:),points_HARRIS{i}(2,:),'ro','MarkerSize',10);
subplot(2,3,3);
imshow(castle{i});
hold on;
plot(points_HESSIAN{i}(1,:),points_HESSIAN{i}(2,:),'bo','MarkerSize',8);
i = 2;
subplot(2,3,4);
imshow(castle{i});
hold on;
plot(feat_SIFT{i}(1,:),feat_SIFT{i}(2,:),'ko','MarkerSize',12);
subplot(2,3,5);
imshow(castle{i});
hold on;
plot(points_HARRIS{i}(1,:),points_HARRIS{i}(2,:),'ro','MarkerSize',10);
subplot(2,3,6);
imshow(castle{i});
hold on;
plot(points_HESSIAN{i}(1,:),points_HESSIAN{i}(2,:),'bo','MarkerSize',8);
%plot(all_feature_points{1}(1,:),all_feature_points{1}(2,:),'k.','MarkerSize',10);

%% Find matches

% pre-allocation of match indices
match_indices = cell(1,N);
matches_HARRIS = cell(1,N);
matches_SURF = cell(1,N);
% pre-allocation of matched point locations
matched_points_left = cell(1,N);
matched_points_right = cell(1,N);
matched_points_HARRIS_left = cell(1,N);
matched_points_HARRIS_right = cell(1,N);
matched_points_SURF_left = cell(1,N);
matched_points_SURF_right = cell(1,N);

% get matched points of image pairs (left-right)
fprintf('computing matches: finished image ');
for i = 1:N
    if i < N
        % find match indices in both images
        match_indices{i} = vl_ubcmatch(descriptors{i},descriptors{i+1});
        % find matched points coordinates in right image
        matched_points_right{i} = all_feature_points{i+1}(:,match_indices{i}(2,:));
    else
        % find match indices in both images
        match_indices{i} = vl_ubcmatch(descriptors{i},descriptors{1});
        % find matched points coordinates in right image
        matched_points_right{i} = all_feature_points{1}(:,match_indices{i}(2,:));
        %pR{i} = feats{1}(1:2,matches{i}(2,:));
    end
    % find matched points coordinates in left image  
    %pL{i} = feats{i}(1:2,matches{i}(1,:));
    matched_points_left{i} = all_feature_points{i}(:,match_indices{i}(1,:));
    fprintf('%d/%d... ', i,N);        
end
fprintf('\nFinished computing matches.\n');



%%
% %% Load images, feature points, descriptors, matches
% tic
% fprintf("reading input...");
% load Images
% load SIFTmatches
% fprintf("done\n");
% toc

%% 2.) Normalized eight-point RANSAC
% Pre-allocate cells
best_matched_points_left = cell(1,N);
best_matched_points_right = cell(1,N);
best_match_indices = cell(1,N);
n_inliers = zeros(1,N);
idx_inliers = cell(1,N);
Fund_mat = zeros(3,3,N);

% Loop over all images
tic;
for i = 1:N
    % perform RANSAC implementation of the normalized 8-point algorithm
    [Fund_mat(:,:,i), idx_inliers{i}, best_matched_points_left{i}, best_matched_points_right{i},asdf{i}]...
        = eight_point_ransac(matched_points_left{i}, matched_points_right{i}, 0.0005, 200);
    % get the indices of the best matches
    best_match_indices{i} = match_indices{i}(:,idx_inliers{i});
    % index stats for printing/plotting
    n_points = size(matched_points_left{i},2);
    n_inliers(i) = length(idx_inliers{i});
    fprintf('loop %d/%d finished.. returned %d/%d inliers.\n', i,N, n_inliers(i), n_points);
end
toc;


% plot the epipolar lines of 10 (random) inliers of image pair with least matches
[~,k] = min(n_inliers);
k = 2;
m = randperm(n_inliers(k),min(n_inliers(k),10)); % pick up to 10 random unique samples
figure;
if k < N
    plotEpilines(castle{k}, castle{k+1}, best_matched_points_left{k}(:,m), best_matched_points_right{k}(:,m), Fund_mat(:,:,k))
else
    plotEpilines(castle{k}, castle{1}, best_matched_points_left{k}(:,m), best_matched_points_right{k}(:,m), Fund_mat(:,:,k))
end

%% 3) Chaining: create point view matrix using BEST matches

% duplicate the match cell
matches_pv = best_match_indices;

point_mat_ind = [];
point_mat_loc = [];
% first two rows of the point-view matrix
point_mat_ind(1:2,:) = matches_pv{1}(1:2,:);


ia = cell(1,N);
ib = cell(1,N);
% loop from second to last imageset
for i = 2:N
    % find indices of mutual points
    [~, ia{i-1}, ib{i-1}] = intersect(point_mat_ind(i,:), matches_pv{i}(1,:));

    % write next row of matrix
    point_mat_ind(i+1,ia{i-1}) = matches_pv{i}(2,ib{i-1});
    % clear all the intersecting matches
    matches_pv{i}(:,ib{i-1}) = [];
    % add the remaining matches to the matrix
    point_mat_ind = horzcat(point_mat_ind, vertcat(zeros(i-1,size(matches_pv{i},2)), matches_pv{i}));
        
end
% check where row 1 and N+1 intersect
[~, ia{i}, ib{i}] = intersect(point_mat_ind(N+1,:), point_mat_ind(1,:));
% add col.s of ia to ib, remove ia
point_mat_ind(:,ib{i}) = point_mat_ind(:,ia{i}) + point_mat_ind(:,ib{i});
point_mat_ind(:,ia{i}) = [];
% now add remaining cols of N+1 to row 1
point_mat_ind(1,:) = max(point_mat_ind(1,:),point_mat_ind(N+1,:));
% % remove row N+1
point_mat_ind(N+1,:) = [];
% % now sort point matrix by cols. of first row (optional)
point_mat_ind(point_mat_ind<1) = NaN;
point_mat_ind = sortrows(point_mat_ind',1)';
point_mat_ind(isnan(point_mat_ind)) = 0;
%%% some duplicate entries remain due to matches
fprintf("done\n")

%% Construct point-coordinate matrix from point-index matrix




%% Plot one random column of the matrix to test it
k = randi(size(point_mat_ind,2));
figure;
pts_SFM = plotPointMatrix(castle, all_feature_points, point_mat_ind, k);

j = 9;
k = randperm(size(best_matched_points_left{j},2),10);
figure;
plotMatches(castle{j},castle{j+1},best_matched_points_left{j}(:,k),best_matched_points_right{j}(:,k));

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
m = 3;
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
    idx_sets = all(point_mat_ind(set,:));
    % match values of sets in point matrix
    pm_sets = point_mat_ind(set, idx_sets);
    
    %%% smaller image sets
    set1 = set(1:(m-1));
    set2 = set(2:m);
    % find indices of current small sets witn non-zero entries
    idx_set1 = all(point_mat_ind(set1,:));
    idx_set2 = all(point_mat_ind(set2,:));
    % match values of small sets in point matrix
    pm_set1 = point_mat_ind(set1, idx_set1);
    pm_set2 = point_mat_ind(set2, idx_set2);
    
    fprintf('sets created...');
    %%% generate list of indexes where set 1 and set 2 are part of sets
    set_mat{i} = zeros(2,size(pm_sets,2));
    for j = 1:size(pm_sets,2)
        set_mat{i}(1,j) = find(ismember(pm_set1(1:(m-1),:)',pm_sets(1:(m-1),j)', 'rows'));
        set_mat{i}(2,j) = find(ismember(pm_set2(1:(m-1),:)',pm_sets(2:m,j)', 'rows'));
    end
    fprintf('set matrix generated....');
    % get image points
    D = [];
    for j = 1:(m-1)
        D(2*j-1:2*j,:) = all_feature_points{set1(j)}(:,pm_set1(j,:));
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
    fprintf('got S&M...');
    Mn{i} = M;
    Sn{i} = S;
    %%%solve for affine ambiguity (copypaste)
    A1 = M(1:2,:);
    L0=pinv(A1'*A1);
    save('M','M')
    lsq_opts = optimoptions('lsqnonlin','Display','off'); % hide msg in prompt
    [L, res(i)] = lsqnonlin(@myfun,L0,[],[],lsq_opts);
    fprintf('Finished LSQ...');
    [C{i},p(i)] = chol(L,'lower');
    if p(i) > 0
        L = nearestSPD(L);
        [C{i},~] = chol(L,'lower');
    end
    
%     fprintf('Finished chol, p = %d...',p(i));
    Mn{i} = M*C{i};
    Sn{i} = pinv(C{i})*S;
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

X = cell(1,N-1);
Y = cell(1,N-1);
Z = cell(1,N-1);
transform = cell(1,N-1);
%figure;
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
fprintf('..done\n');
%% get pointcloud

Sn_cloud = cell2mat(Sn);
color_cloud = cell2mat(imagePoints');

cloud = pointCloud(Sn_cloud', 'Color', color_cloud);

% plot pointcloud
figure;
ax = pcshow(cloud,'MarkerSize', 50);
%ax.XLim = [-1500 1500];
%ax.YLim = [-1500 1500];
%ax.ZLim = [-1500 1500];

%% interpolate the pointcloud with surf...
% get point locs.
[XYZr,xa,~] = unique(round(cloud.Location,0),'rows');
Colorsr = cloud.Color(xa,:);

% ordinary surface plot w/ colors
% Z_mat = [];
% for i=1:size(XYZr,1)
%     Z_mat(500+XYZr(i,1),500+XYZr(i,2)) = XYZr(i,3) + 500;
%     C_mat(500+XYZr(i,1),500+XYZr(i,2),:) = Colorsr(i,:);
% end
% XX = 1:size(Z_mat,1);
% YY = 1:size(Z_mat,2);
% figure;
% surf(Z_mat,C_mat,'EdgeColor','interp');

% simplices for triangulation 
TRIeval = delaunayn(XYZr(:,[2 1]));

% triplot w/ colors
figure;
trisurf(TRIeval,XYZr(:,1),XYZr(:,2),XYZr(:,3),[1:size(XYZr,1)]','FaceColor','flat','EdgeColor','interp')
colormap(double(Colorsr)./256);

%% filter out simplices that are long in x/y direction AND have a large area
ang =zeros(size(TRIeval,1),3);
area = zeros(size(TRIeval,1),1);
si = zeros(size(TRIeval,1),3);
for i = 1:size(TRIeval,1)
    % point indices
    p0 = TRIeval(i,1);
    p1 = TRIeval(i,2);
    p2 = TRIeval(i,3);
    
    % vertex locations in x-y
    v0 = XYZr(p0,1:2);
    v1 = XYZr(p1,1:2);
    v2 = XYZr(p2,1:2);
    
    % simplex lengths
    si(i,1) = sqrt(sum(v0-v1)^2);
    si(i,2) = sqrt(sum(v1-v2)^2);
    si(i,3) = sqrt(sum(v2-v0)^2);
    
    % get angles btwn points in deg
    ang(i,1) = atan2d(abs(det([v2-v0;v1-v0])),dot(v2-v0,v1-v0));
    ang(i,2) = atan2d(abs(det([v0-v1;v2-v1])),dot(v0-v1,v2-v1));
    ang(i,3) = atan2d(abs(det([v1-v2;v0-v2])),dot(v1-v2,v0-v2));
    
    % get area
    area(i,1) = polyarea([v0(1) v1(1) v2(1)],[v0(2) v1(2) v2(2)]);
    % find cg. of each simplex
    % cg(i,:) = [mean([v0(1) v1(1) v2(1)]) mean([v0(2) v1(2) v2(2)])];
end

%% remove some 'wrong' simplices
TRInew = TRIeval;
i = 1;
% while i <= size(TRInew,1)
%     if (min(ang(i,:)) < 40 && area(i) > (mean(area)+1.5*std(area)))
%         TRInew(i,:) = [];
%     end
%     i = i + 1;
% end

% idx_rem = find(min(ang,[],2) < 50 & area>10*mean(area));
% idx_rem3 = find(min(ang,[],2) < 15 & area>2*mean(area));
% idx_rem2 = find(min(ang,[],2) < 5);% & area>4*mean(area));
% TRInew(unique([idx_rem' idx_rem2' idx_rem3']),:) = [];

[simp_rem,~] = find(si>60);
TRInew(simp_rem,:) = [];
% triplot w/ colors
figure;
trisurf(TRInew,XYZr(:,1),XYZr(:,2),XYZr(:,3),[1:size(XYZr,1)]','FaceColor','flat','EdgeColor','flat')
hold on
% for i = 1:size(TRIeval,1)
%     text(cg(i,1),cg(i,2),sprintf('%d',i),'Fontsize',6,'Color','red')
% end
colormap(double(Colorsr)./256);
    