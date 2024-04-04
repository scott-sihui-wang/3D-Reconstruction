I1 = imread('../data/im1.png');
I2 = imread('../data/im2.png');
Corresp = load('../data/someCorresp.mat');
% F = eightpoint(Corresp.pts1, Corresp.pts2, Corresp.M);
F = eightpoint2(Corresp.pts1, Corresp.pts2); % Fundamental matrix F by modified eight-point algorithm
intrinsic = load('../data/intrinsics.mat'); % load intrinsic matrices
E = essentialMatrix(F, intrinsic.K1, intrinsic.K2); % recover essential matrix E
ext1 = [eye(3), zeros(3, 1)]; % camera1's pose: [I | 0]
P1 = intrinsic.K1 * ext1; % camera1' projection matrix: K1*[I | 0]
P2_orig = camera2(E); % 4 candidate poses (extrinsic matrix [R | t]) for camera2
minDist = Inf;
minReproj1 = Inf;
minReproj2 = Inf;
num_valid_pts = 0;
for i = 1: 4
    P2_orig(:, :, i) = intrinsic.K2 * P2_orig(:, :, i); % candidate projection matrix P = K2 * [R | t] for camera2
    p3d = triangulate(P1, Corresp.pts1, P2_orig(:, :, i), Corresp.pts2); % triangulation. p3d are the reconstructed 3D points
    if sum((p3d(:,3)>0),'all') > num_valid_pts % the correct projection matrix have the greatest number of reconstructed points with positive depth
        num_valid_pts = sum((p3d(:,3)>0), 'all');
        P2 = P2_orig(:, :, i);
        p3d = [p3d, ones(size(p3d, 1), 1)];
        pts3d = p3d;
        p1_2d = P1 * (p3d');
        p2_2d = P2_orig(:, :, i) * (p3d');
        p1_2d = p1_2d ./ p1_2d(3, :);
        p2_2d = p2_2d ./ p2_2d(3, :);
        reproj1 = 0;
        reproj2 = 0;
        for j = 1:size(p3d, 1)
            reproj1 = reproj1 + norm(Corresp.pts1(j, :) - p1_2d(1:2, j)'); % calculate the distance between each 2d point and each projected 3d point
            reproj2 = reproj2 + norm(Corresp.pts2(j, :) - p2_2d(1:2, j)');
        end
        reproj1 = reproj1 / size(p3d,1); % mean of the euclidean: reprojection error
        reproj2 = reproj2 / size(p3d,1);
        dist = reproj1 + reproj2;
        minDist = dist;
        minReproj1 = reproj1;
        minReproj2 = reproj2;
    else if sum((p3d(:, 3)>0), 'all') == num_valid_pts 
            % if two projection matrices have equal number of reconstructed
            % points with positive depth, compare their reprojection errors
            % the one with the minimal reprojection error is the correct
            % projection matrix
            p3d = [p3d, ones(size(p3d, 1), 1)];
            p1_2d = P1 * (p3d');
            p2_2d = P2_orig(:, :, i) * p3d';
            p1_2d = p1_2d ./ p1_2d(3, :);
            p2_2d = p2_2d ./ p2_2d(3, :);
            reproj1 = 0;
            reproj2 = 0;
            for j = 1:size(p3d, 1)
                reproj1 = reproj1 + norm(Corresp.pts1(j, :) - p1_2d(1:2, j)');
                reproj2 = reproj2 + norm(Corresp.pts2(j, :) - p2_2d(1:2, j)');
            end
            reproj1 = reproj1 / size(p3d,1);
            reproj2 = reproj2 / size(p3d,1);
            dist = reproj1 + reproj2;
            if dist < minDist
                pts3d = p3d;
                P2 = P2_orig(:, :, i);
                minDist = dist;
                minReproj1 = reproj1;
                minReproj2 = reproj2;
            end
        end
    end
end
fprintf('Reprojection error for image 1: %f\n',minReproj1);
fprintf('Reprojection error for image 2: %f\n',minReproj2);