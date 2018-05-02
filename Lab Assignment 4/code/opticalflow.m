%function F = flow(im1,im2)
% calculates the optical flow between images im1 and im2
%
%INPUT
%- im1: first image (in time) should be in black and white
%- im2: second image (in time) should be in black and white
%- sigma: how much you smooth the image
%
%OUTPUT
%- F: vector of flows
%- ind: indexes of the flow vectors
function [F,ind] = opticalflow(im1,im2,sigma,dp)

% if no images are provided load standard images
if nargin < 1
    im1 = imread('synth1.pgm');
    im2 = imread('synth2.pgm');
    sigma = 1;
    dp = 15;
end

% convert images to double precision
%im1 = double(im1);
%im2 = double(im2);

% divide regions
[h,w] = size(im1);

hDivide = floor(h/dp);
wDivide = floor(w/dp);

%Calculate the center coordinates of the regions
ind = zeros(wDivide,hDivide,2);
ind(:,:,1) = repmat((0:wDivide-1)',1,hDivide)*dp+dp/2;
ind(:,:,2) = repmat((0:hDivide-1),wDivide,1)*dp+dp/2;
% ind(:,:,1) = X;
% ind(:,:,2) = Y;
%Find image derivatives (refer to Gaussian filters in Exercise 1)
[G, Gx, Gy] = gaussian2d(3, 3, sigma);

Ix = conv2(im2, Gx,'same');
Iy = conv2(im2, Gy,'same');
It = conv2((im1-im2),G,'same');

%For every patch find flow vector and store in F
F = zeros(wDivide,hDivide,2);
for i=0:hDivide-1
    for j=0:wDivide-1
        % make a matrix consisting of derivatives along the patch
        A1 = reshape(Ix((i*dp+(1:dp)),(j*dp+(1:dp))),[dp*dp,1]);
        A2 = reshape(Iy((i*dp+(1:dp)),(j*dp+(1:dp))),[dp*dp,1]);
        A = [A1 A2];
        % make b matrix consisting of derivatives in time
        b = reshape(It((i*dp+(1:dp)),(j*dp+(1:dp))),[dp*dp,1]);
        v = inv(A'*A)*A'*b;
        F(i+1,j+1,:) = v;
    end
end

end