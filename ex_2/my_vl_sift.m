function [f,d]= my_vl_sift(img_patha)

Ia = imread(img_patha);

Ia = single(rgb2gray(Ia)) ;

points_inv = scale_inv(img_patha);

points_inv2 = [zeros(size(points_inv,1),1) points_inv];

points_inv2 = transpose(points_inv2);

points_inv2(transpose([1 2 3 4]),:) = points_inv2([3 2 4 1],:);


[f,d] = vl_sift(Ia,'frames',points_inv2,'orientations') ;

