% the Lukas Kanade Tracker:
% the initial points in the first frams are tracked. In the video
% 'tracked.avi' this is shown, where yellow dots are the ground truth and
% pink dots are the tracked points
%%%You can also not follow this instrcution and implement the tracker
%%%according to your own interpretation!
function [pointsx, pointsy] = LKtracker(p,im,sigma)

%pre-alocate point locations and image derivatives
pointsx = zeros(size(im,3),size(p,2));
pointsy = zeros(size(im,3),size(p,2));
pointsx(1,:) = p(1,:);
pointsy(1,:) = p(2,:);
%fill in starting points

It=zeros(size(im) - [0 0 1]);
Ix=zeros(size(im) - [0 0 1]);
Iy=zeros(size(im) - [0 0 1]);

%calculate the gaussian derivative
G = 
Gd = 

%find x,y and t derivative
for i=1:size(im,3)-1
    Ix(:,:,i)=
    Iy(:,:,i)=
    It(:,:,i)=
end

% writerObj = VideoWriter('test.avi');
% open(writerObj);

for num = 1:size(im,3)-1 % iterating through images
    for i = 1:size(p,2) % iterating throught points
        % make a matrix consisting of derivatives around the pixel location
        x =                      %%%center of the patch
        y =                      %%%center of the patch
        A1 = 
        A2 =
        A = 
        % make b matrix consisting of derivatives in time
        b = 
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        v = 
        pointsx(num+1,i) = 
        pointsy(num+1,i) = 
    end
% %     %     figure(1)
% %     %     imshow(im(:,:,num),[])
% %     %     hold on
% %     %     plot(pointsx(num,:),pointsy(num,:),'.y') %tracked points
% %     %     plot(p(num*2-1,:),p(num*2,:),'.m')  %ground truth
% %     %     frame = getframe;
% %     %     writeVideo(writerObj,frame);
end
%close(writerObj);


end
