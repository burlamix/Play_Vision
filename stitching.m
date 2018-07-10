function [S_stitched, T, b, c, S] = stitching(S, set_mutuals)

N = size(S,2);

T = zeros(3,3,N);
b = zeros(1,N);
c = zeros(3,N);
% transform of image set 1 to 1
T(:,:,1) = eye(3); % rotation
b(1) = 1; % scale
c(:,1) = 0; % translation
%figure;
% 3d transform
for i = 1:(N-1)
    % want to find the transform between mutual 3D points of current and next set
    X = S{i}(:,set_mutuals{i}(1,:))';
    Y = S{i+1}(:,set_mutuals{i}(2,:))';
    % calc. a 3D mapping between these MUTUAL points of current and next set
    [~,~,transform]=procrustes(X,Y);
    % now transform ALL points in the next set with the found 3D mapping
    S{i+1} = (transform.b.*S{i+1}'*transform.T + transform.c(1,:).*ones(length(S{i+1}),3))';
    % save the transform parameters in a better accessible way
    T(:,:,i+1) = transform.T;
    b(i+1) = transform.b;
    c(:,i+1) = transform.c(1,:)';
%     % plot
%     % all points
%     plot3(Sn{i}(1,:),Sn{i}(2,:),Sn{i}(3,:),'kx',Sp{i+1}(1,:),Sp{i+1}(2,:),Sp{i+1}(3,:),'c.',Sn{i+1}(1,:),Sn{i+1}(2,:),Sn{i+1}(3,:),'cx', 'MarkerSize', 5);
%     hold on
%     % points used for 3D transform
%     plot3(X{i}(:,1),X{i}(:,2),X{i}(:,3),'rx',Y{i}(:,1),Y{i}(:,2),Y{i}(:,3),'b.',Z{i}(:,1),Z{i}(:,2),Z{i}(:,3),'bx', 'MarkerSize', 10);
%     hold off
%     title(sprintf('plot %d: images %d, %d, %d, %d', i, sets(i,1),sets(i,2),sets(i,3),sets(i,4)))
%     pause;
end
% Now put the individual cells of S for each view into one matrix
S_stitched = cell2mat(S);

return
end