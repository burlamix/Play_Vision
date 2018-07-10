function [magnitude , orientation] = gradmag(image_path , sigma)

 I = imread(image_path);
 I = rgb2gray(I);
 I = im2double(I);


gd = gaussianDer(@gaussian,sigma);

drx = conv2(I,gd' ,'same');
dry = conv2(I,gd,'same');

orientation = atan2( dry, drx );
%imshow(orientation);

magnitude = sqrt( drx .*drx + dry.*dry ) ;


[Gmag,Gdir] = imgradient(I);

%figure(1)
%imshow(Gmag);


%figure(2)
%imshow(magnitude)


end

