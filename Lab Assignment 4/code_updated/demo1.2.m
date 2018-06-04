%function demo1
%Shows a demo of the flow function on two images. Also produces an animated
%gif
function demo1()

%% Show the flow in the synthesized image
clf();
im1 = imread('synth1.pgm');
im2 = imread('synth2.pgm');
[F,ind] = opticalflow(im1,im2,1);
imshow(im1)
hold on
quiver(ind(:,:,1),ind(:,:,2),F(:,:,1),F(:,:,2),'b');

%pause process
display('Press any key to continue')
pause()

%% Show the flow in the sphere image
clf();
im1 = imread('sphere1.ppm');
im2 = imread('sphere2.ppm');

ig1 = rgb2gray(im1);
ig2 = rgb2gray(im2); 
[F,ind] = opticalflow(ig1,ig2,1);

imshow(im1)
hold on
quiver(ind(:,:,2),ind(:,:,1),F(:,:,1),F(:,:,2),'m');

%% Produce animated gif of sphere
f = getframe;
im = frame2im(f);
[imind,cm] = rgb2ind(im,256);
imwrite(imind,cm,'sphere','gif', 'Loopcount',inf);

imshow(im2)
quiver(ind(:,:,2),ind(:,:,1),F(:,:,1),F(:,:,2),'m');
f = getframe;
im = frame2im(f);
[imind,cm] = rgb2ind(im,256);
imwrite(imind,cm,'sphere','gif','WriteMode','append');
end


