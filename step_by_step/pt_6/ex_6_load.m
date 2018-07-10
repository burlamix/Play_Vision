
clear;clc;close all

N = 16;

%% features
fprintf("Extracting features:\n")
for i = 1:N
%fname_img = sprintf('TeddybearPNG/obj02_%03d.png',i);
fname_har = sprintf('TeddybearPNG/obj02_%03d.png.haraff.sift',i);
fname_hes = sprintf('TeddybearPNG/obj02_%03d.png.hesaff.sift',i);
[x{i,1}, y{i,1}, a{i,1}, b{i,1}, c{i,1}, desc{i,1}] = load_features(fname_har);
[x{i,2}, y{i,2}, a{i,2}, b{i,2}, c{i,2}, desc{i,2}] = load_features(fname_hes);
X{i} = [x{i,1} x{i,2}];
Y{i} = [y{i,1} y{i,2}];
A{i} = [a{i,1} a{i,2}];
B{i} = [b{i,1} b{i,2}];
C{i} = [c{i,1} c{i,2}];
Desc{i} = [desc{i,1} desc{i,2}];
Feat{i} = [X{i}; Y{i}; A{i}; B{i}; C{i}];
fprintf('%d/%d...', i,N);
end

%% matches
matches = cell(1,N);
fprintf("\nComputing Matches\n")
for i = 1:N
    if i < N
        matches{i} = vl_ubcmatch(Desc{i}, Desc{i+1});
    else
        matches{i} = vl_ubcmatch(Desc{i}, Desc{1});
    end
    fprintf('%d/%d...', i,N);
end

%% ransac
fprintf("\nRansac\n")
for i = 1:N
    if i < N
        [X_best{i}, ind_inliers{i}, pix_dist{i}, p_L{i}, p_R{i}] = ransac_func(Feat{i}, Feat{i+1}, matches{i}, 10);
    else
        [X_best{i}, ind_inliers{i}, pix_dist{i}, p_L{i}, p_R{i}] = ransac_func(Feat{i}, Feat{1}, matches{i}, 10);
    end
    newmatches{i} = matches{i}(:,ind_inliers{i});
end

%% point view matrix


matches_pv = matches;

point_mat = [];

point_mat(1:2,:) = matches_pv{1}(1:2,1:end);
offset = 0;
for i = 2:15
    [~, ia, ib] = intersect(matches_pv{i-1}(2,:), matches_pv{i}(1,:));
    point_mat(i+1,offset+ia) = matches_pv{i}(2,ib);
    offset = size(point_mat,2);
    matches_pv{i}(:,ib) = [];
    point_mat = horzcat(point_mat, vertcat(zeros(i-1,size(matches_pv{i},2)), matches_pv{i}));
end

i = 16; offset = 0;
[~, ia, ib] = intersect(matches_pv{i-1}(2,:), matches_pv{i}(1,:));
point_mat(i,offset+ia) = matches_pv{i}(2,ib);
%offset = size(point_mat,2);
%matches_pv{i}(:,ib) = [];
%point_mat = horzcat(point_mat, vertcat(zeros(i-1,size(matches_pv{i},2)), matches_pv{i}));

fprintf("done")