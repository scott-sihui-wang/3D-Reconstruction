I1 = imread('../data/im1.png');
I2 = imread('../data/im2.png');
Corresp = load('../data/someCorresp.mat');
M = max(size(I1, 1), size(I1, 2));
F = eightpoint(Corresp.pts1, Corresp.pts2, M);
epipolarMatchGUI(I1, I2, F);