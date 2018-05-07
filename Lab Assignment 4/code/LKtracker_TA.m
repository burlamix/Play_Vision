% the Lukas Kanade Tracker:
% the initial points in the first frams are tracked. In the video
% 'tracked.avi' this is shown, where yellow dots are the ground truth and
% pink dots are the tracked points
function [pointsx, pointsy] = LKtracker_TA(p,im,sigma)

%pre-alocate point locations and image derivatives
pointsx = zeros(size(im,3),size(p,2));
pointsy = zeros(size(im,3),size(p,2));

It=zeros(size(im) - [0 0 1]);
Ix=zeros(size(im) - [0 0 1]);
Iy=zeros(size(im) - [0 0 1]);

%fill in starting points
pointsx(1,:) = p(1,:);
pointsy(1,:) = p(2,:);

%calculate the gaussian derivative
G = fspecial('gaussian',[1 2*ceil(3*sigma)+1],sigma)
Gd = gaussianDer(G,sigma)

%find x,y and t derivative
for i=1:size(im,3)-1
    Ix(:,:,i)=conv2(conv2(im(:,:,i),Gd,'same'),G','same');
    Iy(:,:,i)=conv2(conv2(im(:,:,i),Gd','same'),G,'same');
    It(:,:,i)=im(:,:,i+1)-im(:,:,i);
end

% writerObj = VideoWriter('test.avi');
% open(writerObj);

for num = 1:size(im,3)-1 % iterating through images
    for i = 1:size(p,2) % iterating throught points
        x = min(max(round(pointsx(num,i)),8),size(im,2)-7);
        y = min(max(round(pointsy(num,i)),8),size(im,1)-7);
        % make a matrix consisting of derivatives around the pixel location
        A1 = Ix(y-7:y+7,x-7:x+7,num);
        A2 = Iy(y-7:y+7,x-7:x+7,num);
        A = [A1(:),A2(:)];
        % make b matrix consisting of derivatives in time
        b = It(y-7:y+7,x-7:x+7,num);
        b = b(:);
        v = pinv(A'*A) * A' * double(b);
        pointsx(num+1,i) = pointsx(num,i)-v(1);
        pointsy(num+1,i) = pointsy(num,i)-v(2);
    end
   % figure(1)
   % imshow(im(:,:,num),[])
   % hold on
   % plot(pointsx(num,:),pointsy(num,:),'.y') %tracked points
   % plot(p(num*2-1,:),p(num*2,:),'.m')  %ground truth
   % frame = getframe;
  %  writeVideo(writerObj,frame);
end
% close(writerObj);


end
