clear;clc;
close all;

% Load images
busleft = imread('../left.jpg');
busright = imread('../right.jpg');

% Convert to grayscale single
busleft_G = single(rgb2gray(busleft))./255;
busright_G = single(rgb2gray(busright))./255;

% Get SIFT matches
[feat_l, desc_l] = vl_sift(busleft_G);
[feat_r, desc_r] = vl_sift(busright_G);
matches = vl_ubcmatch(desc_l,desc_r);


points_left = feat_l(1:2,matches(1,:));
points_right = feat_r(1:2,matches(2,:));
pix_thr = 10;

[inliers, pix_dist, pL, pR, T] = ransac_affine(points_left, points_right, pix_thr);

figure(1);
plotMatches2(busleft, busright, pL, pR)
%% Stitching

height_L = size(busleft,1);
width_L = size(busleft,2);
height_R = size(busright,1);
width_R = size(busright,2);

Tform_old = maketform('affine',T);
Tform_new = affine2d(T);

Tform_L = maketform('affine',eye(3));

[xR_limits, yR_limits] = outputLimits(Tform_new, [1 width_R], [1 height_R]);

xmax = ceil(xR_limits(2));
ymax = max(ceil(yR_limits(2)), height_L); 

xdata = [1 xR_limits(2)];
ydata = [1 ymax];

R_left = imtransform(busleft, Tform_L, 'bicubic', 'Xdata', xdata, 'Ydata', ydata,  'FillValues', NaN);
R_right = imtransform(busright, Tform_old, 'bicubic', 'Xdata', xdata, 'Ydata', ydata, 'FillValues', NaN);

R(:,:,:,1)=R_left;
R(:,:,:,2)=R_right;
Z = nanmean(R,4);
figure(2);
imshow(Z./255);