function points = scale_inv(img)

I = imread(img);
I = rgb2gray(I);
I = im2double(I);

img_height  = size(I,1);
img_width   = size(I,2);

%default variable
num_sigma = 12;
sigma_array = ((1.2).^(0:num_sigma));

harris_point = zeros(0,3);

%calculating the harris key point for each different sigma
for i= 1:num_sigma
    [l,c] = harris_s(img,sigma_array(i));
    n = size(l,1);
    harris_point(end+1:end+n,:) = [l,c,repmat(i,[n,1])];
end
%harris_point is a matrix, where each row is a keypoiny the value of the row 
%are the following:( x_axis, y_axis, number of the sigma 1,2,3, ...)


% compute scale-normalized laplacian operator
laplace_snlo = zeros(img_height,img_width,num_sigma);
for i=1:num_sigma
    s_L = sigma_array(i);   % scale
    laplace_snlo(:,:,i) = s_L*s_L*imfilter(I,fspecial('log', floor(6*s_L+1), s_L),'replicate');
end

% salvo i valori di ogni key point più alti
    n   = size(harris_point,1);
    cpt = 0;
    points = zeros(n,3);
    for i=1:n
        l = harris_point(i,1);
        c = harris_point(i,2);
        s = harris_point(i,3);
        val = laplace_snlo(l,c,s);
        if s>1 && s<num_sigma
            if val>laplace_snlo(l,c,s-1) && val>laplace_snlo(l,c,s+1)
                cpt = cpt+1;
                points(cpt,:) = harris_point(i,:);
            end
        elseif s==1
            if val>laplace_snlo(l,c,2)
                cpt = cpt+1;
                points(cpt,:) = harris_point(i,:);
            end
        elseif s==num_sigma
            if val>laplace_snlo(l,c,s-1)
                cpt = cpt+1;
                points(cpt,:) = harris_point(i,:);
            end
        end
    end
    
points(cpt+1:end,:) = [];

%{
imshow(I)
hold on;
    
n =size(points,1)

for i = 1:n
    hold on;
    plot(points(i,2), points(i,1), 'r*', 'LineWidth',1 , 'MarkerSize', 6);
    hold on;
end
%}
   
end

