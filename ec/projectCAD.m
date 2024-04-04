CAD = load('../data/PnP.mat');
P = estimate_pose(CAD.x, CAD.X);
[K, R, t] = estimate_params(P);
p3Dh = [CAD.X; ones(1, size(CAD.X, 2))];
p2Dh = P * p3Dh;
p2D = p2Dh(1:2, :)./p2Dh(3, :);

figure(1);
imshow(CAD.image);
hold on;
plot(CAD.x(1, :), CAD.x(2, :), 'co',  'MarkerSize', 5, 'MarkerFaceColor', 'c');
plot(p2D(1, :), p2D(2, :), 'yo', 'MarkerSize', 16, 'LineWidth', 1);
hold off;

figure(2);
v3d = CAD.cad.vertices;
v3d_t = v3d * R';
trimesh(CAD.cad.faces, v3d_t(:, 1), v3d_t(:, 2), v3d_t(:, 3), 'EdgeColor', 'k'); % original mesh, with rotated 3d points

figure(3);
v3d_h = [v3d, ones(size(v3d, 1), 1)];
v2d_h = v3d_h * P';
v2d = v2d_h(:, 1: 2) ./ v2d_h(:, 3);
imshow(CAD.image);
hold on;
patch('Faces', CAD.cad.faces, 'Vertices', v2d, 'FaceColor', 'green', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
hold off;

