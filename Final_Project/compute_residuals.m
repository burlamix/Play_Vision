function residuals = compute_residuals(L, M)
%returns a matrix containing the residuals
%computed for every camera view point
%INPUT
%- L: the matrix L that will be minimized for Ai*L*Ai' = eye(2)
%
%OUTPUT
%- Dif: a matrix (n x 4) containing the residuals for the n cameras

%pre-allocate the Dif matrix
residuals = zeros(size(M,1)/2,4);

%compute the residuals
for i = 1:size(M,1)/2
    Ai = M(i*2-1:i*2,:);
    D = Ai*L*Ai' - eye(2);
    residuals(i,:) = D(:);
end
return
end