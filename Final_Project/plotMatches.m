function []  = plotMatches(image_left, image_right, p_L, p_R)
%PLOTMATCHES Summary of this function goes here
%   Detailed explanation goes here
% Define Colorlist
c = [1 1 0; 1 0 1; 0 1 1; 1 0 0; 0 1 0; 0 0 1; 1 1 1; 0 0 0];
m = size(c,1); % nr. of colors

N = size(p_L, 2);
% join two equally sized images
image_big = [image_left image_right];

figure;
% plot lines between images
imshow(image_big);
title('Selected points on images'); 
for i = 1:N
    hold on
    plot([p_L(1,i),size(image_right,2)+p_R(1,i)],[p_L(2,i),p_R(2,i)],...
        '--', 'Color',c(mod(i,m)+1,:),'LineWidth',0.5)
    plot([p_L(1,i),size(image_right,2)+p_R(1,i)],[p_L(2,i),p_R(2,i)],...
        '.', 'Color',c(mod(i,m)+1,:),'MarkerSize',20)
end

end

