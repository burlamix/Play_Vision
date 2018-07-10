function F =  ImageDerivatives(I , sigma , type)


dx = gaussianDer(@gaussian,sigma);
dy = dx';


%second derivative
x = -3*sigma : 3*sigma;
x = (x.^2 - sigma^2) / (sigma^4);
ddx = x .* gaussian(sigma);
ddy = ddx';

switch type
    case 'x'
        F = conv2(I,dx,'same');
    case 'y'
        F = conv2(I,dy,'same');
    case 'xx'
        F = conv2(I,ddx,'same');    
    case 'yy'
        F = conv2(I,ddy,'same');        
    case 'xy'
        F = conv2(ddx,ddy,I,'same'); 
    otherwise
        F = conv2(ddy,ddx,I,'same');
end


end
