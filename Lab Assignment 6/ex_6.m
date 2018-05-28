clear; clc; close(gcf);

%% 0a) Nr. of Images
N = 20;

%% 0b) Pre-allocating images
teddy = cell(1,N);
teddy_gray = cell(1,N);
feats = cell(1,N);
desc = cell(1,N);
%% script shell
for i = 19:N
    
    command1 = sprintf('extract_features/extract_features.ln -haraff -i TeddyBearPNG/obj02_%03d.png -sift -pca extract_features/harhessift.basis',i);
    command2 = sprintf('extract_features/extract_features.ln -hesaff -i TeddyBearPNG/obj02_%03d.png -sift -pca extract_features/harhessift.basis',i);
    
    system(char(command1));
    system(char(command2));
end


%%
