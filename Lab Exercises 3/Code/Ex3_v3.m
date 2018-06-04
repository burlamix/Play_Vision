clear;clc;
close all;
%% Load images and get matched points
N = 6;
for i = 1:N
    boat{i} = imread(sprintf('../boat/img%d.pgm',i));
    boat{i} = single(boat{i})./255;
    [feat{i}, desc{i}] = vl_sift(boat{i});
end

for i = 1:(N-1)
    matches{i} = vl_ubcmatch(desc{i}, desc{i+1});
    points_left{i} = feat{i}(1:2, matches{i}(1,:));
    points_right{i} = feat{i+1}(1:2, matches{i}(2,:));
end

%% Apply RANSAC
pix_thr = 1;
T = zeros(3,3,N);
T(:,:,1) = eye(3);
for i = 1:(N-1)
    [inliers, pix_dist, pL{i}, pR{i}, T(:,:,i+1)] = ransac_affine(points_left{i}, points_right{i}, pix_thr);
end

% figure(1);
% plotMatches2(busleft, busright, pL, pR)
%% Stitching

figure(1);
stitchImages(boat, T);
