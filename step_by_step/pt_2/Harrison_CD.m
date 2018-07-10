function [outputArg1,outputArg2] = Harrison_CD(img_patha,img_pathb)

Ia = imread(img_patha);
Ib = imread(img_pathb);

Ibp = im2double(rgb2gray(Ib));
Iap = im2double(rgb2gray(Ia));

Ia = single(rgb2gray(Ia)) ;
Ib = single(rgb2gray(Ib)) ;

%[fa, da] = vl_sift(Ia) ;
%[fb, db] = vl_sift(Ib) ;

[fa, da] = my_vl_sift(img_patha) 
[fb, db] = my_vl_sift(img_pathb) ;

%{
imshow(Ibp)
hold on;
n = size(fb,2)
for i = 1:n
    hold on;
    plot(fb(1,i), fb(2,i), 'r*', 'LineWidth',1 , 'MarkerSize', 6);
    hold on;
end
%}
perm = randperm(size(fb,2)) ;
sel = perm(1:50) ;
h1 = vl_plotframe(fb(:,sel)) ;
h2 = vl_plotframe(fb(:,sel)) ;
set(h1,'color','k','linewidth',3) ;
set(h2,'color','y','linewidth',2) ;




[matches, scores] = vl_ubcmatch(da, db) ;


[drop, perm] = sort(scores, 'descend') ;
matches = matches(:, perm) ;
scores  = scores(perm) ;

figure(1) ; clf ;
imagesc(cat(2, Ia, Ib)) ;
axis image off ;
vl_demo_print('sift_match_1', 1) ;

figure(2) ; clf ;
imagesc(cat(2, Ia, Ib)) ;

xa = fa(1,matches(1,:)) ;
xb = fb(1,matches(2,:)) + size(Ia,2) ;
ya = fa(2,matches(1,:)) ;
yb = fb(2,matches(2,:)) ;

hold on ;
h = line([xa ; xb], [ya ; yb]) ;
set(h,'linewidth', 1, 'color', 'b') ;

vl_plotframe(fa(:,matches(1,:))) ;
fb(1,:) = fb(1,:) + size(Ia,2) ;
vl_plotframe(fb(:,matches(2,:))) ;
axis image off ;

vl_demo_print('sift_match_2', 1) ;

end

