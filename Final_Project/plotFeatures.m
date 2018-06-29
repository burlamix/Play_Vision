function plotFeatures(Image_set, i, points_SIFT, points_HARRIS, points_HESSIAN)
%PLOTFEATURES Summary of this function goes here
%   Detailed explanation goes here
N = size(points_SIFT,2);
if i > N
    i = mod(i,N);
end

% plot features of different methods 
figure;
subplot(1,3,1);
imshow(Image_set{i});
hold on;
plot(points_SIFT{i}(1,:),points_SIFT{i}(2,:),'ko','MarkerSize',8);
title('SIFT feature points')
subplot(1,3,2);
imshow(Image_set{i});
hold on;
plot(points_HARRIS{i}(1,:),points_HARRIS{i}(2,:),'ro','MarkerSize',8);
title('HARRIS feature points')
subplot(1,3,3);
imshow(Image_set{i});
hold on;
plot(points_HESSIAN{i}(1,:),points_HESSIAN{i}(2,:),'bo','MarkerSize',8);
title('HESSIAN feature points')


end

