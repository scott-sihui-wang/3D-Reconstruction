I1 = imread('../data/im1.png');
I2 = imread('../data/im2.png');
data = load('../data/someCorresp.mat');
M = max(size(I1, 1), size(I1, 2));
F = eightpoint(data.pts1, data.pts2, M);
intrinsic = load('../data/intrinsics.mat');
E = essentialMatrix(F, intrinsic.K1, intrinsic.K2)