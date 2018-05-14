clear; clc; close(gcf);

%% Pre-allocating images
castle = cell(1,19);
castle_gray = cell(1,19);
feats = cell(1,19);
desc = cell(1,19);
%% Load Images and Perform SIFT
tic;
for i = 1:19
    fname = sprintf('model_castle/8ADT%d.JPG',8585+i);
    castle{1,i} = imread(fname);
    
    castle_gray{i} = single(rgb2gray(castle{1,i}))./255;
    [feats{i}, desc{i}] = vl_sift(castle_gray{i}, 'PeakThresh', 0.01, 'EdgeThresh',10);
    Xfeats = feats{i}(1,:);
    Yfeats = feats{i}(2,:);
    fprintf('finished image %d/19\n', i);        
%     hold off
%     imshow(castle{i});
%     hold on
%     scatter(Xfeats,Yfeats);
%     pause(0.1)
toc;    
end
matches = cell(1,18);
for i = 1:18
    matches{i} = vl_ubcmatch(desc{i},desc{i+1});
end
toc;


save SIFTmatches feats desc matches
save Images castle castle_gray
clear;
%% Load
tic
fprintf("reading input...");
load Images
load SIFTmatches
fprintf("done\n");
toc
%% RANSAC
pix_thr = 10;
tic;
for i = 1:18
    
      [X{i}, ninliers, pix_dist, pL_best{i}, pR_best{i}] =ransac_func(feats{i}, feats{i+1}, matches{i},pix_thr); 
     %xL_best
     disp(i)
     %m1 = X(1); m2 = X(2); m3 = X(3); m4 = X(4); t1 = X(5); t2 = X(6);
end
 toc;
 
%% structure from motion 
% for 2 img at time (no idea how for more that 2 as write on the pdf ?!)
%scatter3(M(:,1),M(:,2),M(:,3))
% not sure if it works...
%also probably there is a smart way to put all together the point and
%select from it
for i = 1:18

    D = [xL_best{i}; yL_best{i}; xR_best{i}; yR_best{i}];

    [U,W,VT] = svd(D);


    U3 = U(:,1:3);
    V = VT';
    V3 = VT(:,1:3);
    W3 = W(1:3,1:3);

    % motion and structure
    M = U3*sqrt(W3);
    S = sqrt(W3)*(V3');
    
    
    %scatter3(S(1,:),S(2,:),S(3,:))
    %scatter3(M(:,1),M(:,2),M(:,3))
end


