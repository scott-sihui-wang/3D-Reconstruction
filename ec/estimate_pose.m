function P = estimate_pose(x, X)
% ESTIMATE_POSE computes the pose matrix (camera matrix) P given 2D and 3D
% points.
%   Args:
%       x: 2D points with shape [2, N]
%       X: 3D points with shape [3, N]
sz = size(x, 2);
sol_mat = zeros(sz * 2, 12);
for i = 1: sz
    sol_mat(2 * i - 1, :) = [X(1, i), X(2, i), X(3, i), 1, 0, 0, 0, 0, -x(1, i) * X(1, i), -x(1, i) * X(2, i), -x(1, i) * X(3, i), -x(1, i)];
    sol_mat(2 * i, :) = [0, 0, 0, 0, X(1, i), X(2, i), X(3, i), 1, -x(2, i) * X(1, i), -x(2, i) * X(2, i), -x(2, i) * X(3, i), -x(2, i)];
end
[~,~,V] = svd(sol_mat);
P = (reshape(V(:, end), [4, 3]))';