clear;clc;close(gcf);

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

% RANSAC
N = 2000; %nr. of loops
P = 20; %nr. of sample pairs
inliers_best = 0;
for j = 1:N
    p = datasample(matches,P,2,'Replace',false); %second axis, unique samples
    A = []; b = [];
    for i = 1:P % compute A matrix
        x_L(i) = feat_l(1,p(1,i));
        y_L(i) = feat_l(2,p(1,i));
        x_R(i) = feat_r(1,p(2,i));
        y_R(i) = feat_r(2,p(2,i));
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
    inliers = length(find(pix_dist<10)); % nr. of inliers
    if (inliers > inliers_best)
        inliers_best = inliers;
        X_best = X;
        xL_best = x_L; yL_best = y_L; xR_best = x_R; yR_best = y_R;
    end
    
    if (inliers_best == P)
        break
    end
end
X = X_best;
m1 = X(1); m2 = X(2); m3 = X(3); m4 = X(4); t1 = X(5); t2 = X(6);



busleft_M = insertMarker(busleft,[xL_best;yL_best]');
busright_M = insertMarker(busright,[xR_best;yR_best]');
%imshowpair(busleft_M,busright_M,'montage')
% join two images
busbig = [busright_M;...
    zeros(size(busleft_M,1)-size(busright_M,1),size(busright_M,2),3)];
busbig = [busleft_M busbig];

% plot lines
imshow(busbig);
for i = 1:P
    hold on
    plot([xL_best(i),size(busleft_M,2)+xR_best(i)],[yL_best(i),yR_best(i)],'r')
end
pause(0.3)
close(gcf)
inliers_best

%%%%%%%%%%%%%%% STITCHING
T = [m1 m2 t1;...
    m3 m4 t2;...
    0 0 1];

height_L = size(busleft_G,2);
width_L = size(busleft_G,1);
height_R = size(busright_G,2);
width_R = size(busright_G,1);

Tform_old = maketform('affine',T');
Tform_new = affine2d(T');

Tform_R = maketform('affine',eye(3));

[xL_limits, yL_limits] = outputLimits(Tform_new, [1 width_L], [1 height_L]);

xmin = floor(min(xL_limits(1),0));
xmax = ceil(max(xL_limits(2),min(width_L,width_R)));
ymin = floor(min(yL_limits(1),0));
ymax = ceil(max(yL_limits(2),max(height_L,height_R))); 

height_comb = xmax-xmin;
width_comb = ymax-ymin;

%panoramaView = imref2d([height_comb width_comb], xL_limits, yL_limits);


xdata = [xmin xmax];
ydata = [ymin ymax];

%newImsize = [-xL_limits(1)+max(xL_limits(2),height_R), -yL_limits(1)+max(yL_limits(2),width_R)];


R_right = imtransform(busright_G, Tform_R, 'bicubic', 'Xdata', xdata, 'Ydata', ydata, 'FillValues', NaN);
R_left = imtransform(busleft_G, Tform_old, 'bicubic', 'Xdata', xdata, 'Ydata', ydata, 'FillValues', NaN);

R(:,:,1)=R_right;
R(:,:,2)=R_left;
Z = nanmean(R,3);
imshow(Z);



%imshowpair(R,busright,'montage')