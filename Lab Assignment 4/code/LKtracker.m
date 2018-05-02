% the Lukas Kanade Tracker:
% the initial points in the first frams are tracked. In the video
% 'tracked.avi' this is shown, where yellow dots are the ground truth and
% pink dots are the tracked points
%%%You can also not follow this instrcution and implement the tracker
%%%according to your own interpretation!
function [pointsx, pointsy, Vx, Vy] = LKtracker(p_set,im_set,sigma,dp)


% define image sizes
[h,w,N] = size(im_set);

% nr. of points per image
n = size(p_set,2);

%pre-allocate point locations 
pointsx = zeros(N,n);
pointsy = zeros(N,n);

% initial point location
pointsx(1,:) = p_set(1,:);
pointsy(1,:) = p_set(2,:);

%pre-allocate velocities
Vx = zeros(N,n);
Vy = zeros(N,n);

%pre-allocate image derivatives (N-1 frames due to It)
Ix=zeros(size(im_set) - [0 0 1]);
Iy=zeros(size(im_set) - [0 0 1]);
It=zeros(size(im_set) - [0 0 1]);

%calculate the gaussian derivative
[G, Gx, Gy] = gaussian2d(3, 3, sigma);

%find x,y and t derivative
for i=1:N-1 % start at image 2, image 1 has no time derivative
    Ix(:,:,i)=conv2(im_set(:,:,i+1), Gx,'same');
    Iy(:,:,i)=conv2(im_set(:,:,i+1), Gy,'same');
    It(:,:,i)=conv2((im_set(:,:,i)-im_set(:,:,i+1)),G,'same');
end

% writerObj = VideoWriter('test.avi');
% open(writerObj);

for num = 1:N-1 % iterating through images (N)
    for i = 1:n % iterating through points (columns->n)
        x = round(p_set(2*num-1,n));        % x-center of the patch, forced integer
        y = round(p_set(2*num,n));          % y-center of the patch, forced integer
        
        xmin = max(1,x-(dp-1)/2); % cut-off x-window at 0
        xmax = min(x+(dp-1)/2,w); % cut-off x-window at w
        ymin = max(1,y-(dp-1)/2); % cut-off y-window at 0
        ymax = min(y+(dp-1)/2,h); % cut-off y-window at h
        
        xrange = xmin:xmax; % x-window
        yrange = ymin:ymax; % y-window
        win_size = length(xrange)*length(yrange);
        
        % make A matrix consisting of derivatives around the pixel location
        A1 = reshape(Ix(yrange,xrange,num),[win_size,1]); %y=rows, x=cols!!
        A2 = reshape(Iy(yrange,xrange,num),[win_size,1]);
        A = [A1 A2];
        % make b matrix consisting of derivatives in time
        b = reshape(It(yrange,xrange,num),[win_size,1]);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        v = inv(A'*A)*A'*b;
        Vx(num+1,i) = v(1);
        Vy(num+1,i) = v(2);
        pointsx(num+1,i) = pointsx(num,i)+Vx(num+1,i);
        pointsy(num+1,i) = pointsy(num,i)+Vy(num+1,i);
    end
    %     figure(1)
    %     imshow(im(:,:,num),[])
    %     hold on
    %     plot(pointsx(num,:),pointsy(num,:),'.y') %tracked points
    %     plot(p(num*2-1,:),p(num*2,:),'.m')  %ground truth
    %     frame = getframe;
    %     writeVideo(writerObj,frame);
end
%close(writerObj);


end
