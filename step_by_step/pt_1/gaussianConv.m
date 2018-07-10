function imOut =  gaussianConv(image_path , sigma_x , sigma_y)

n=sigma_x*6+1;

 I = imread(image_path);
 I = rgb2gray(I);
 I = im2double(I);

% my implementation
g1 = gaussian(sigma_x);
g2 = gaussian(sigma_y);

imOut1 = conv2(g1,g2,I,'same');

imshow(imOut1);

%%% matlab implementation
gm=fspecial('gaussian',n,sigma_x);
imOut2 = imgaussfilt(I,sigma_x);


%imOut2 = conv2(I,gm,'same');



end
