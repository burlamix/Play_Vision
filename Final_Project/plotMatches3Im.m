function []  = plotMatches3Im(image_left, image_center, image_right, p_L, p_C, p_R)
%PLOTMATCHES Summary of this function goes here
%   Detailed explanation goes here

N = size(p_L, 2);
width = size(image_left,2);

% Insert markers at specified points
%image_left = insertMarker(image_left, p_L', 'x', 'Size', round(size(image_left,2)/200));
%image_center = insertMarker(image_center, p_C', 'x', 'Size', round(size(image_left,2)/200));
%image_right = insertMarker(image_right, p_R', 'x', 'Size', round(size(image_left,2)/200));

% join three equally sized images
image_big = [image_left image_center image_right];

% plot lines between images
imshow(image_big);
for i = 1:N
    hold on
    plot([p_L(1,i),p_C(1,i)+width, p_R(1,i)+2*width],[p_L(2,i),p_C(2,i), p_R(2,i)],'g.','MarkerSize',14,'LineWidth',2)
    plot([p_L(1,i),p_C(1,i)+width, p_R(1,i)+2*width],[p_L(2,i),p_C(2,i), p_R(2,i)],'r')
end

end