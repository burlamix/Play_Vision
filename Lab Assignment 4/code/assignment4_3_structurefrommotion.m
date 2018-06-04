clear; clc;
[D, Dc] = readMeasurementMatrix(); % points and centered points

% paper D
Dx = Dc(1:2:202,:);
Dy = Dc(2:2:202,:);
Dp = [Dx; Dy];

% svd
[U,W,V] = svd(Dc);

U3 = U(:,1:3);
V3 = V(:,1:3);
W3 = W(1:3,1:3);

D3 = U3*W3*V3';

% motion and structure
M = U3*sqrt(W3);
S = sqrt(W3)*(V3');


% plot structure
figure(1);
plot3(S(1,:),S(2,:),S(3,:),'.r')
grid on

% plot motion
figure(2);
plot3(M(:,1),M(:,2),M(:,3),'.b')
grid on