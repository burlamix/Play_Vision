clear; clc; close(gcf);

%% 0a) Nr. of Images
N = 20;

%% 0b) Pre-allocating images
teddy = cell(1,N);
teddy_gray = cell(1,N);
feats = cell(1,N);
desc = cell(1,N);
%% script shell
for i = 1:N
    
    command1 = sprintf('extract_features/extract_features.ln -haraff -thres 13000 -i TeddyBearPNG/obj02_%03d.png -sift -pca extract_features/harhessift.basis',i);
    command2 = sprintf('extract_features/extract_features.ln -hesaff -thres 1300 -i TeddyBearPNG/obj02_%03d.png -sift -pca extract_features/harhessift.basis',i);
    
    system(char(command1));
    system(char(command2));
end



%% get feature and match
for i = 11:N
    loadFeatures('TeddyBearPNG/obj02_%03d.png.haraff.sift')
end