function [G, Gx, Gy] =gaussian2d(x, y, sigma)
  % [x,y] is grid size, sigma speaks for itself
 [X, Y]=meshgrid(ceil(-x/2):floor(x/2), ceil(-y/2):floor(y/2));
 G=exp(-X.^2/(2*sigma^2)-Y.^2/(2*sigma^2));
 G=G./sum(G(:)); % at N = inf--> amp = 1/(2 sigma^2 pi)
 Gx = -X/sigma^2.*G;
 Gy = -Y/sigma^2.*G;
  end