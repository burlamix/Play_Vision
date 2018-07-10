function F = computeFundamental(p_L, p_R)
%COMPUTEFUNDAMENTAL Summary of this function goes here
%   p_L, p_R = 2xN matrix (or 3xN, doesn't really matter)

x_L = p_L(1,:)'; % transpose necessary for construction of A
y_L = p_L(2,:)';
x_R = p_R(1,:)';
y_R = p_R(2,:)';

A = [x_L.*x_R x_L.*y_R x_L y_L.*x_R y_L.*y_R y_L x_R y_R ones(size(x_L))];

% SVD of A (F = eigenvector corr. to smallest eigenvalue of A'*A)
%%% Reader method
[~, ~, V] = svd(A);
F=reshape(V(:,9),[3 3]);

%%% Matlab method 
% [eigv,~]=eigs(A'*A,1,'sm');
% F = reshape(eigv,[3 3]);

% SVD of F
[Uf, Df, Vf] = svd(F);

% Force F to singularity
Df(3,3) = 0;
F = Uf*Df*Vf';
return
end

