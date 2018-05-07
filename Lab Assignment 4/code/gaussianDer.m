function Gd =gaussianDer(G , sigma)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

%x = -3*sigma : 3*sigma;
%Gd = (exp((-x.^2)/(2*sigma^2))  /  ((sigma^3)*sqrt(2*pi))) ;
%Gd = Gd ./ (sum(Gd));

%x = -3*sigma : 3*sigma;
%x= -(x ./ (sigma^2));

%Gd = x .* G;

x = -3*sigma:3*sigma;
Gd = -(x)/(sigma^2).*G;

%Gd = Gd ./ (sum(Gd));


end

