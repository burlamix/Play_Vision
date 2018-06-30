function E = bundleAdjustment(D, M_comb, m, n)

E = 0;

% Reshape  to get M & S
M = reshape(M_comb(1:m*6), [2*m 3]);
S = reshape(M_comb(end-3*n+1:end), [3 n]);

P = M * S;


for i = 1:m
    for j = 1:n
        E = E + sqrt((D(i*2-1, j) - P(i*2-1, j))^2 + (D(i*2, j) - P(i*2, j))^2);
    end
end

end