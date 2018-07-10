function G = gaussian(sigma)
    %hsize = 9;
    %x = -floor(hsize/2):floor(hsize/2);
    %x = meshgrid(l,l); %1x9
    x = -3*sigma:3*sigma
    G = 1/(sigma*sqrt(2*pi)) * exp(-x.^2/(2*sigma^2));
    G = G/sum(G); % normalize;
end