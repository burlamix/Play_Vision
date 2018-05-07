% this function gives a demo of the Lucas Kanade tracker. The sum of the
% euclidean distance between the tracked points and the groundtruth is
% plotted for every frame.

%function demo13()
clear;clc; close all
%load points
%Points = textread('..\model_house\measurement_matrix.txt');
[Points, Points_norm] = readMeasurementMatrix();

Pointsx = Points(1:2:202,:);
Pointsy = Points(2:2:202,:);

for num = 1:101
    imageLoc = ['model_house/frame' num2str(num, '%08d') '.jpg'];
    im = double(imread(imageLoc))/255;
    if num == 1
        Imf=zeros(size(im,1),size(im,2),101);
    end
    Imf(:,:,num)=im;
end

%track points
%[pointsx,pointsy]=LKtracker_TA(Points,Imf,1);
[pointsx,pointsy]=LKtracker(Points,Imf,1,15);

% save('Xpoints','pointsx')
% save('Ypoints','pointsy')

%% plot error
for num = 1:101
    imshow(Imf(:,:,num));
    hold on
    plot(Points(num*2-1,:),Points(num*2,:),'b.');
    plot(pointsx(num,:),pointsy(num,:),'r.');
    %quiver(pointsx(num,:),pointsy(num,:),Vx(num,:),Vy(num,:));
    pause(0.01)
end

%% euclidean distance per frame
dis_x = abs(Points(1:2:size(Points,1),:)-pointsx);
dis_y = abs(Points(2:2:size(Points,1),:)-pointsy);
figure(2)
eudis=sqrt((dis_x).^2+(dis_y).^2);
LS=sum(eudis,2);
plot(LS)
xlabel('image #')
ylabel('sum of LS-error')
%end