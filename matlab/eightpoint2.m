function F = eightpoint2(pts1, pts2)

sz = size(pts1,1);
assert(size(pts2,1) == sz); % test if input is correct
%m1 = mean(pts1);
%m2 = mean(pts2);
pts = [pts1; pts2]; % put all input points together
m = mean(pts); % compute the centroid
d = sqrt(2)/mean(sqrt(sum((pts - mean(pts)).^2, 2))); % sqrt(2) / mean distance to the centroid
%d1 = 1.0/mean(sqrt(sum((pts1 - m1).^2, 2)));
%d2 = 0.9/mean(sqrt(sum((pts2 - m2).^2, 2)));
pts1 = [pts1 ones(sz, 1)]; % homogeneous coordinates
pts2 = [pts2 ones(sz, 1)];
%T1 = [d1, 0, -m1(1)*d1; 0, d1, -m1(2)*d1; 0, 0, 1];
%T2 = [d2, 0, -m2(1)*d2; 0, d2, -m2(2)*d2; 0, 0, 1];
T = [d, 0, -m(1)*d; 0, d, -m(2)*d; 0, 0, 1]; % normalization

%pts1 = pts1 * (T1');
%pts2 = pts2 * (T2'); % the commented lines with m1, m2, d1, d2, T1, T2
%implemented another feasible normalization technique. The reprojection error could
%be even lower with this normalization technique, if you find a good
%configuration for the parameters.

pts1 = pts1 * T';
pts2 = pts2 * T';

sol_mat = zeros(sz, 9);
for i = 1:sz
    sol_mat(i,:)=[pts2(i,1)*pts1(i,1) pts2(i,1)*pts1(i,2) pts2(i,1) pts2(i,2)*pts1(i,1) pts2(i,2)*pts1(i,2) pts2(i,2) pts1(i,1) pts1(i,2) 1];
end
[~,~,V] = svd(sol_mat);
F = reshape(V(:,end),[3,3]);
[U,S,V] = svd(F);
F = U * diag([S(1,1), S(2,2), 0]) * V'; % rank 2 constraint
F = refineF(F, pts1, pts2);
%F = T2' * F * T1;
F = T' * F * T; % denormalization