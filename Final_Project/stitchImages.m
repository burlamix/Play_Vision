function stitchImages( image_cell, AffineTformMat)
%STITCHIMAGES Summary of this function goes here
%   Detailed explanation goes here

height = size(image_cell{1},1);
width = size(image_cell{1},2);
N = length(image_cell);

T = AffineTformMat;
Tform = cell(1,N);
x_limits = zeros(N,2);
y_limits = zeros(N,2);
T_current = eye(3);
for i = 1:N
    % Update cumulative T matrix
    T_current = T(:,:,i)*T_current;
    Tform{i} = maketform('affine',T_current);
    % Get output limits
    Tform_limit = affine2d(T_current);
    [x_limits(i,:), y_limits(i,:)] = outputLimits(Tform_limit, [1 width], [1 height]);
end

Tform_limit = affine2d(T_current);
[x_limits, y_limits] = outputLimits(Tform_limit, [1 width], [1 height]);


xmin = min(x_limits(:,1));
xmax = max(x_limits(:,2));
ymin = min(y_limits(:,1));
ymax = max(y_limits(:,2));

xdata = [floor(xmin) ceil(xmax)];
ydata = [floor(ymin) ceil(ymax)];


for i = 1:N
    R(:,:,:,i) = imtransform(image_cell{i}, Tform{i}, 'bicubic', 'Xdata', xdata, 'Ydata', ydata, 'FillValues', NaN);
end


Z = nanmean(R,4);
imshow(Z./255);


end

