function [Sn, Mn, D, set_mutuals, C, p] = getStructurefromMotion(m, point_mat_ind, all_feature_points)
%STITCHING Summary of this function goes here
%   Detailed explanation goes here

N = size(point_mat_ind,1);

% build set of m+1 image rows
sets = [1:N]';
for i = 2:(m+1)
    sets(:,i) = sets(:,i-1) + 1;
    for j= 1:N
        if sets(j,i) > N
            sets(j,i) = sets(j,i) - N;
        end
    end
end

set_mutuals = cell(1,N);
Mn = cell(1,N);
Sn = cell(1,N);
C = cell(1,N);
p = zeros(1,N);
D = cell(1,N);
% generate each m-image block
for i = 1:N
    %%% big image set
    set = sets(i,:);
    % find indices of current big set witn non-zero entries
    idx_sets = all(point_mat_ind(set,:));
    % match values of sets in point matrix
    pm_sets = point_mat_ind(set, idx_sets);
    
    % current image set
    set_current = set(1:m);
    % next image set, of which the current set needs an overlap with
    set_next = set(2:(m+1));
    % find indices of current and next set with non-zero entries
    idx_set_current = all(point_mat_ind(set_current,:));
    idx_set_next = all(point_mat_ind(set_next,:));
    % get the match indices of current and next set
    pm_set_current = point_mat_ind(set_current, idx_set_current);
    pm_set_next = point_mat_ind(set_next, idx_set_next);
    
    %%% generate list of mutual indices of current and next set
    set_mutuals{i} = zeros(2,size(pm_sets,2));
    % for each image, the point matrix must have at least 2 columns with m+1 or
    % more matches in a row
    for j = 1:size(pm_sets,2)
        % check where the transposed pm rows of the current set match the
        % transposed pm rows of the m+1 set
        set_mutuals{i}(1,j) = find(ismember(pm_set_current(1:(m-1),:)',pm_sets(1:(m-1),j)', 'rows'));
        % do the same for tne next set
        set_mutuals{i}(2,j) = find(ismember(pm_set_next(1:(m-1),:)',pm_sets(2:m,j)', 'rows'));
    end
    % mutual set needs at least 2 mutual points for the 3D transform
    if size(set_mutuals{i},2) < 2
        fprintf('Error: Not enough consecutive points in set %d:\n',i);
        set
        error('Try lowering the RANSAC threshold for finding matches.');
        return
    end
    % now get the image coordinates of the current set of m images
    n_pts = size(pm_set_current,2);
    D{i} = zeros(2*m,n_pts);
    for j = 1:m
        % cannot slice the cell array so have to loop through it...
        D{i}(2*j-1:2*j,:) = all_feature_points{set_current(j)}(:,pm_set_current(j,:));
    end
    
    %%% structure from motion
    % Center D
    N_view = size(D{i}, 1) / 2; N_point = size(D{i}, 2);

    Dn  = D{i} - repmat(sum(D{i},2)/n_pts, 1, n_pts);
    % SVD of normalized D
    [U, W, V] = svd(Dn);
    % Force to rank 3
    U3 = U(:,1:3);
    V3 = V(:,1:3);
    W3 = W(1:3,1:3);
    % Recreate structure and motion
    M = U3*sqrt(W3);
    S = sqrt(W3)*V3';
    
    %%% now solve for affine ambiguity:
    % find 3x3 matrix C such that axes of M are orthogonal
    A1 = M(1:2,:);
    L0=pinv(A1'*A1);
    % apply non-lin lst squares to solve for L
    lsq_opts = optimoptions('lsqnonlin','Display','off'); % hide msg in prompt
    L = lsqnonlin(@(x)compute_residuals(x, M),L0,[],[],lsq_opts);
    % C = LL^T, obtain C from Choletski decomposition
    [C{i},p(i)] = chol(L,'lower');
    if p(i) < 1
        Mn{i} = M*C{i};
        Sn{i} = pinv(C{i})*S;
    else
        fprintf('Warning: Matrix L of image set %d is not SPD, p = %d\n',i,p(i));
        % if L is not SPD, just use the original M and S
        Mn{i} = M;
        Sn{i} = S;
    end
    
    
    %boundle

    MS_0 = [Mn{i}(:); Sn{i}(:)];
    %options = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', 'InitDamping', 1e2, 'ScaleProblem', 'jacobian', 'StepTolerance', 1e-16, 'OptimalityTolerance', 1e-16, 'FunctionTolerance', 1e-16, 'Display', 'iter');
    options = optimoptions(@fminunc, 'Display', 'off');
    
    MS = fminunc(@(x)bundleAdjustment(Dn, x, N_view, N_point), MS_0, options);
    
    % Get back S & M
    M_BA = reshape(MS(1:N_view*6), [2*N_view 3]);
    S_BA = reshape(MS(end-3*N_point+1:end), [3 N_point]);

    % Put in final array
    Sn{i} = S_BA;
    Mn{i} = M_BA;
    
    fprintf('Finished image set %d/%d\n',i,N);

    
end


end

