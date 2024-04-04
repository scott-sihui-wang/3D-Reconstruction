function F = eightpoint(pts1, pts2, M)
% eightpoint:
%   pts1 - Nx2 matrix of (x,y) coordinates
%   pts2 - Nx2 matrix of (x,y) coordinates
%   M    - max (imwidth, imheight)

% Q2.1 - Todo:
%     Implement the eightpoint algorithm
%     Generate a matrix F from correspondence '../data/some_corresp.mat'
sz = size(pts1,1);
assert(size(pts2,1) == sz); % test if input is correct
pts1 = pts1/M;
pts2 = pts2/M; % rescale for normalization
pts1 = [pts1 ones(sz,1)];
pts2 = [pts2 ones(sz,1)]; % homogenous coordination

sol_mat = zeros(sz, 9);
for i = 1:sz
    sol_mat(i,:)=[pts2(i,1)*pts1(i,1) pts2(i,1)*pts1(i,2) pts2(i,1) pts2(i,2)*pts1(i,1) pts2(i,2)*pts1(i,2) pts2(i,2) pts1(i,1) pts1(i,2) 1];
end
[~,~,V] = svd(sol_mat);
F = reshape(V(:,end),[3,3]);
[U,S,V] = svd(F);
F = U * diag([S(1,1), S(2,2), 0]) * V'; % rank 2 constraint
F = refineF(F, pts1, pts2);
T = diag([1/M, 1/M, 1]); % unscale
F = T' * F * T;