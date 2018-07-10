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
function [F,ind] = opticalflow(im1,im2,sigma)

% if no images are provided load standard images
if nargin < 1
    im1 = imread('synth1.pgm');
    im2 = imread('synth2.pgm');
    sigma = 1;
end

% convert images to double precision
im1 = double(im1);
im2 = double(im2);

% devide regions
[h,w] = size(im1);

hDevide = floor(h/15);
wDevide = floor(w/15);

%Calculate the center coordinates of the regions
ind = zeros(hDevide,wDevide,2);
ind(:,:,1) = repmat((0:wDevide-1)',1,hDevide)*15+7.5;
ind(:,:,2) = repmat((0:hDevide-1),wDevide,1)*15+7.5;

%Find image derivatives (refer to Gaussian filters in Exercise 1)
G = 
Gd = 

Ix = 
Iy = 
It =

%For every patch find flow vector and store in F
F = zeros(hDevide,wDevide,2);
for i=0:hDevide-1
    for j=0:wDevide-1
        % make a matrix consisting of derivatives along the patch
        A1 =
        A2 =
        A =
        % make b matrix consisting of derivatives in time
        b =
        v =
        F(i+1,j+1,:) = v;
    end
end

end