% A test script using templeCoords.mat
%
% Write your code here
%
I1 = imread('../data/im1.png');
I2 = imread('../data/im2.png');
Corresp = load('../data/someCorresp.mat'); % step 1: load from someCorresp.mat
F = eightpoint(Corresp.pts1, Corresp.pts2, Corresp.M); % step 2: compute fundamental matrix
%F = eightpoint2(Corresp.pts1, Corresp.pts2);
Coords = load('../data/templeCoords.mat'); % step 3: load from templeCoords.mat
pts2 = zeros(size(Coords.pts1));
for i = 1: size(pts2)
    pts2(i,:) = epipolarCorrespondence(I1, I2, F, Coords.pts1(i,:)); % get corresponding points by epipolarCorrespondences.m
end
intrinsic = load('../data/intrinsics.mat');% step 4: load intrinsic matrix, compute E
E = essentialMatrix(F, intrinsic.K1, intrinsic.K2);
% step 5: compute P1 and candidate P2
ext1 = [eye(3), zeros(3, 1)]; % cam1's extrinsic matrix is [I | 0]
P1 = intrinsic.K1 * ext1;
P2_orig = camera2(E);
minDist = Inf;
minReproj1 = Inf;
minReproj2 = Inf;
num_valid_pts = 0;
%step 7: choose the correct one from the candidate P2
for i = 1: 4
    P2_orig(:, :, i) = intrinsic.K2 * P2_orig(:, :, i);
    p3d = triangulate(P1, Coords.pts1, P2_orig(:, :, i), pts2);% step 6: run triangulation with 4 candidates
    if sum((p3d(:,3)>0),'all') > num_valid_pts
        num_valid_pts = sum((p3d(:,3)>0), 'all');
        P2 = P2_orig(:, :, i);
        p3d = [p3d, ones(size(p3d, 1), 1)];
        pts3d = p3d;
        p1_2d = P1 * (p3d');
        p2_2d = P2_orig(:, :, i) * p3d';
        p1_2d = p1_2d ./ p1_2d(3, :);
        p2_2d = p2_2d ./ p2_2d(3, :);
        reproj1 = 0;
        reproj2 = 0;
        for j = 1:size(p3d, 1)
            reproj1 = reproj1 + norm(Coords.pts1(j, :) - p1_2d(1:2, j)');
            reproj2 = reproj2 + norm(pts2(j, :) - p2_2d(1:2, j)');
        end
        reproj1 = reproj1 / size(p3d,1);
        reproj2 = reproj2 / size(p3d,1);
        dist = reproj1 + reproj2;
        minDist = dist;
        minReproj1 = reproj1;
        minReproj2 = reproj2;
    else if sum((p3d(:, 3)>0), 'all') == num_valid_pts
            p3d = [p3d, ones(size(p3d, 1), 1)];
            p1_2d = P1 * (p3d');
            p2_2d = P2_orig(:, :, i) * p3d';
            p1_2d = p1_2d ./ p1_2d(3, :);
            p2_2d = p2_2d ./ p2_2d(3, :);
            reproj1 = 0;
            reproj2 = 0;
            for j = 1:size(p3d, 1)
                reproj1 = reproj1 + norm(Coords.pts1(j, :) - p1_2d(1:2, j)');
                reproj2 = reproj2 + norm(pts2(j, :) - p2_2d(1:2, j)');
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
%step 8: plot
plot3(pts3d(:, 1), pts3d(:, 2), pts3d(:, 3), 'r.');
axis equal;
rotate3d on;
R1 = intrinsic.K1 \ P1(1: 3,1: 3);
t1 = intrinsic.K1 \ P1(:, 4);
R2 = intrinsic.K2 \ P2(1: 3,1: 3);
t2 = intrinsic.K2 \ P2(:, 4);
% step 9: save the results
% save extrinsic parameters for dense reconstruction
save('../data/extrinsics.mat', 'R1', 't1', 'R2', 't2');
