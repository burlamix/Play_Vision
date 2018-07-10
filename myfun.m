%function Dif = myfun(L)
%returns a matrix containing the residuals
%computed for every camera view point
%INPUT
%- L: the matrix L that will be minimized for Ai*L*Ai' = eye(2)
%
%OUTPUT
%- Dif: a matrix (n x 4) containing the residuals for the n cameras
function Dif = myfun(L)

%load the saved transformation matrix M
load('M');

%pre-alocate the Dif matrix
Dif = zeros(size(M,1)/2,4);

%compute the residuals
for i = 1:size(M,1)/2
    Ai = M(i*2-1:i*2,:);
    D = Ai*L*Ai' - eye(2);
    Dif(i,:) = D(:);
end