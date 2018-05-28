function [x, y, a, b, c, desc] = load_features(file)
%
%Displays affine regions on the image

fid = fopen(file, 'r');
dim=fscanf(fid, '%f',1);
if dim==1
    dim=0;
end
nb=fscanf(fid, '%d',1);
feat = fscanf(fid, '%f', [5+dim, inf]);
fclose(fid);

x = feat(1,:);
y = feat(2,:);
a = feat(3,:);
b = feat(4,:);
c = feat(5,:);
desc = feat(6:end,:);

end