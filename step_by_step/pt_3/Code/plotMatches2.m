function []  = plotMatches2(image_left, image_right, p_L, p_R)
%PLOTMATCHES Summary of this function goes here
%   Detailed explanation goes here

N = size(p_L, 2);
% Insert markers at specified points
image_left = insertMarker(image_left, p_L', 'x', 'Size', round(size(image_left,2)/200));
image_right = insertMarker(image_right, p_R', 'x', 'Size', round(size(image_left,2)/200));

% join two unequally sized images

xR = size(image_right,2);
% yR = size(image_right,1);
% dx = size(image_left,2) - size(image_right,2);
dy = size(image_left,1) - size(image_right,1);

image_big = horzcat(image_left, vertcat(image_right, zeros(dy,xR,3)));
% plot lines between images
imshow(image_big);
    hold on
for i = 1:N
    % plot each line separately
    plot([p_L(1,i),size(image_left,2)+p_R(1,i)],[p_L(2,i),p_R(2,i)],'r')
end

end

