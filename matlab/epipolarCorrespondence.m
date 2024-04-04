function [pts2] = epipolarCorrespondence(im1, im2, F, pts1)
% epipolarCorrespondence:
%   Args:
%       im1:    Image 1
%       im2:    Image 2
%       F:      Fundamental Matrix from im1 to im2
%       pts1:   coordinates of points in image 1
%   Returns:
%       pts2:   coordinates of points in image 2
%
sz = size(pts1, 1);
pts1 = [pts1, ones(sz, 1)];
epi = F * pts1'; % epipolar line constraint for pts2
% According to http://www.cs.umd.edu/~djacobs/CMSC733/stereo.pdf, we should
% compare intensity using SSD
im1 = rgb2gray(im1);
im2 = rgb2gray(im2);
patchSize = 10;
searchSize = 10;
pts1_int = round(pts1);
% note that size(img, 1) is along y-axis, pts(2) is the corresponding y
% coordination
p1_bound = [min(patchSize, pts1_int(2) - 1), min(patchSize, size(im1, 1) - pts1_int(2)), min(patchSize, pts1_int(1) - 1), min(patchSize, size(im1, 2) - pts1_int(1))];
dist = Inf;
for offset = - min(searchSize, pts1_int(1) - 1) : min(searchSize, size(im1, 2) - pts1_int(1))
    x = pts1_int(1) + offset;
    pts2_cur = [(x), -((epi(1) * x + epi(3)) / epi(2)), 1];
    pts2_cur = round(pts2_cur);
    p2_bound = [min(patchSize, pts2_cur(2) - 1), min(patchSize, size(im2, 1) - pts2_cur(2)), min(patchSize, pts2_cur(1) - 1), min(patchSize, size(im2, 2) - pts2_cur(1))];
    patch_bound = [min(p1_bound(1), p2_bound(1)), min(p1_bound(2), p2_bound(2)), min(p1_bound(3), p2_bound(3)), min(p1_bound(4), p2_bound(4))];
    patch1 = im1((pts1_int(2) - patch_bound(1)): (pts1_int(2) + patch_bound(2)), (pts1_int(1) - patch_bound(3)): (pts1_int(1) + patch_bound(4)));
    patch2 = im2((pts2_cur(2) - patch_bound(1)) :(pts2_cur(2) + patch_bound(2)), (pts2_cur(1) - patch_bound(3)): (pts2_cur(1) + patch_bound(4)));
    dist_cur = sum(sum((patch2 - patch1).^2));
    if dist_cur < dist
        dist = dist_cur;
        pts2 = pts2_cur(1:2);
    end
end
