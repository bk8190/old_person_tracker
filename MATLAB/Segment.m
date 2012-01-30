function [gmdist] = Segment(img)

if ~exist('img','var')
    run_as_script = true
    clear all;close all;
    [img ~] = LoadData('images/',1);
end

MAX_DIST = 7;

N_GAUSSIANS= 15;

HIST_NBINS = 80;

IMG_FIGURE = 1;
HIST_FIGURE = 2;
DIST_FIGURE = 3;
CLUSTERS_FIGURE = 4;

% Remove NAN values
batman = isnan(img);
img(batman) = MAX_DIST;


% Show a histogram
notatmax = img( img<(MAX_DIST-.05) );
[counts bins] = hist(notatmax, HIST_NBINS);

counts = counts/sum(counts);

samples = imresize(img, .1, 'nearest');
samples = samples( samples<(MAX_DIST-.05) );

options = statset('MaxIter', 500, 'Display', 'final');
gmdist = gmdistribution.fit(samples(:), N_GAUSSIANS, 'CovType', 'diagonal', 'Regularize', .001, 'Options', options);

idx = cluster(gmdist, img(:));
idx = reshape(idx, size(img,1), size(img,2));

figure(HIST_FIGURE);
plot(bins,counts)

figure(IMG_FIGURE)
outimg = img./MAX_DIST;
imshow(outimg);

figure(DIST_FIGURE)
x = (0:.01:MAX_DIST)';
p = pdf(gmdist,x);
plot(x,p);

figure(CLUSTERS_FIGURE)
imshow(label2rgb(idx, 'lines'))
%end