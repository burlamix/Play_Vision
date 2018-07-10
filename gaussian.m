function G = gaussian(sigma)

x = -3*sigma : 3*sigma;

G = (exp((-x.^2)/(2*sigma^2))  /  (sigma*sqrt(2*pi))) ;

G = G ./ (sum(G));

end

