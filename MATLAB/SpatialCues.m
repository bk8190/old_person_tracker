
%-----------------------------------------------%
function [hC vC] = SpatialCues(im)
g = fspecial('gauss', [13 13], sqrt(13));
dy = fspecial('sobel');
vf = conv2(g, dy, 'valid');
sz = size(im);

vC = zeros(sz(1:2));
hC = vC;

vC = abs(imfilter(im, vf, 'symmetric'));
hC = abs(imfilter(im, vf', 'symmetric'));
