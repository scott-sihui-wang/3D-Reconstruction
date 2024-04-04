function dispM = get_disparity(im1, im2, maxDisp, windowSize)
% GET_DISPARITY creates a disparity map from a pair of rectified images im1 and
%   im2, given the maximum disparity MAXDISP and the window size WINDOWSIZE.
im1 = im2double(im1);
im2 = im2double(im2); 
% with this conversion the output will be more similar to the sample provided in the hand-out
% without this conversion, the result will be a little be fragmented.
dispM = ones(size(im1)) * maxDisp;
dist = ones(size(im1)) * inf;
mask = ones(windowSize, windowSize);
for d = 0: dispM
    dist_cur = conv2((im1 - imtranslate(im2, [d, 0])).^2, mask, 'same');
    dispM(dist_cur < dist) = d;
    dist = min(dist, dist_cur);
end
