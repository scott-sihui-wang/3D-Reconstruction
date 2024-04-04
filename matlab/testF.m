I1 = imread('../data/im1.png');
I2 = imread('../data/im2.png');
data = load('../data/someCorresp.mat');
M = max(size(I1, 1), size(I1, 2)); % M is the maximum of the image's width and height
F = eightpoint(data.pts1, data.pts2, M);
disp(F)
displayEpipolarF(I1,I2,F);