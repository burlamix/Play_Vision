clear; clc; close(gcf);

%% Load Images and Perform SIFT
tic;
for i = 1:19
    fname = sprintf('model_castle/8ADT%d.JPG',8585+i);
    castle{i} = imread(fname);
    castle_gray{i} = single(rgb2gray(castle{i}))./255;
    [feats{i}, desc{i}] = vl_sift(castle_gray{i}, 'PeakThresh', 0.01, 'EdgeThresh',10);
    Xfeats = feats{i}(1,:);
    Yfeats = feats{i}(2,:);
    fprintf('finished image %d/19', i);        
%     hold off
%     imshow(castle{i});
%     hold on
%     scatter(Xfeats,Yfeats);
%     pause(0.1)
    
end
toc;
for i = 1:18
    matches{i} = vl_ubcmatch(desc{i},desc{i+1});
end
toc;
%% RANSAC
pix_thr = 10;

for i = 1:18
    
      [X{i}, xL_best{i}, yL_best{i}, xR_best{i}, yR_best{i}] =ransac_castle(feats{i}, feats{i+1}, matches{i},pix_thr) 
     xL_best
     %m1 = X(1); m2 = X(2); m3 = X(3); m4 = X(4); t1 = X(5); t2 = X(6);
end
 toc;
 
%% structure from motion 
% for 2 img at time (no idea how for more that 2 as write on the pdf ?!)
%scatter3(M(:,1),M(:,2),M(:,3))
% not sure if it works...
%also probably there is a smart way to put all together the point and
%select from it
for i = 1:18

    D = [xL_best{i}; yL_best{i}; xR_best{i}; yR_best{i}];

    [U,W,VT] = svd(D);


    U3 = U(:,1:3);
    V = VT';
    V3 = VT(:,1:3);
    W3 = W(1:3,1:3);

    % motion and structure
    M = U3*sqrt(W3);
    S = sqrt(W3)*(V3');
    
    
    %scatter3(S(1,:),S(2,:),S(3,:))
    %scatter3(M(:,1),M(:,2),M(:,3))
end


%% %%%%%%%%%%%%%
% 
% m = 25; % nr. of sample pairs
% v = 0.15; % prob of selecting outlier
% N = ceil((log10(1-0.99)/log10(1-(1-v)^m))) % nr. of loops
% pix_thr = 10;
% 
% feats_prev = feats{1};
% feats_curr = feats{2};
% inliers_best = 0;
% for j = 1:N
%     p = datasample(matches,m,2,'Replace',false); %second axis, unique samples
%     A = []; b = [];
%     for i = 1:m % compute A matrix
%         x_L(i) = feats_prev(1,p(1,i));
%         y_L(i) = feats_prev(2,p(1,i));
%         x_R(i) = feats_curr(1,p(2,i));
%         y_R(i) = feats_curr(2,p(2,i));
%         A = [A;...
%             x_L(i) y_L(i) 0 0 1 0;...
%             0 0 x_L(i) y_L(i) 0 1];
%         b = [b;...
%             x_R(i);...
%             y_R(i)];
%     end
%     X = pinv(A)*b; % m1;m2;m3;m4;t1;t2
%     m1 = X(1); m2 = X(2); m3 = X(3); m4 = X(4); t1 = X(5); t2 = X(6);
%     % Estimated pixels on right image
%     X_R = x_L.*m1+y_L.*m2+t1;
%     Y_R = x_L.*m3+y_L.*m4+t2;
%     pix_dist = sqrt(abs(x_R-X_R).^2+abs(y_R-Y_R).^2); % pairwise L2 dist
%     inliers = length(find(pix_dist<pix_thr)); % nr. of inliers
%     if (inliers > inliers_best)
%         inliers_best = inliers;
%         X_best = X;
%         xL_best = x_L; yL_best = y_L; xR_best = x_R; yR_best = y_R;
%     end
%     
%     if (inliers_best == m)
%         break
%     end
% end
% X = X_best;
% j