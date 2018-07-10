clear; clc; close(gcf);

fr1 = imread('sphere1.ppm');
fr2 = imread('sphere2.ppm');


s1 = imread('synth1.pgm');
s2 = imread('synth2.pgm');

im1 = im2double(rgb2gray(fr1));
im2 = im2double(rgb2gray(fr2));

im3 = im2double(s1);
im4 = im2double(s2);

[F, ind] = opticalflow(im1, im2, 0.5, 10);


figure()
imshow(fr1);
pause;
imshow(fr2);
hold on;
% draw the velocity vectors
pause;
quiver(ind(:,:,1)',ind(:,:,2)',F(:,:,1),F(:,:,2)) %x,y->v,u in image coords