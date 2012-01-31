function Segment(in_img)

if ~exist('in_img','var')
    run_as_script = true
    clear all;close all;
    [in_img ~] = LoadData('images/',12);
end

total_time = tic();

img = imresize(in_img, .3, 'nearest');

MAX_DIST = 7;
N_GAUSSIANS= 13;
HIST_NBINS = 80;

IMG_FIGURE = 1;
HIST_FIGURE = 2;
GMDIST_FIGURE = 3;
CLUSTERS_FIGURE = 4;
GRAPHCUT_FIGURE = 5;

% Remove NAN values
batman = isnan(img);
img(batman) = MAX_DIST;

sz = size(img);

% Show a histogram
notatmax = img( img<(MAX_DIST-.05) );
[counts bins] = hist(notatmax, HIST_NBINS);
counts = counts/sum(counts);

samples = imresize(img, .3, 'nearest');
% figure(1)
% imshow(samples/MAX_DIST)
% pause;
samples = samples( samples<(MAX_DIST-.05) );
[samplecounts samplebins] = hist(samples, HIST_NBINS);

disp('Fitting distribution')
time = tic();
options = statset('MaxIter', 500, 'Display', 'final');
gmdist = gmdistribution.fit(samples(:), N_GAUSSIANS, 'CovType', 'diagonal', 'Regularize', .001, 'Options', options);
elapsedtime = toc(time);
disp(['EM completed in ' num2str(elapsedtime) ' seconds.'])


% figure(HIST_FIGURE);
% plot(bins,counts/sum(counts), 'b-')
% hold on
% plot(samplebins,samplecounts/sum(samplecounts), 'r-')
% 
% % figure(GMDIST_FIGURE)
% x = (1/HIST_NBINS:MAX_DIST/HIST_NBINS:MAX_DIST)';
% p = pdf(gmdist,x);
% plot(x,p/sum(p), 'k-');
% title('Histogram of image (blue) and sampled image (red) with Gaussian PDF in black')
% % title('PDF of Gaussian model')
% 
% figure(IMG_FIGURE)
% imshow(img/MAX_DIST);
% title('Image')
% 
% time = tic();
% figure(CLUSTERS_FIGURE)
% idx = cluster(gmdist, img(:));
% idx = reshape(idx, size(img,1), size(img,2));
% imshow(label2rgb(idx, 'lines'))
% title('Clusters detected from EM')
% elapsedtime = toc(time);
% disp(['Clustering completed in ' num2str(elapsedtime) ' seconds.'])




k = N_GAUSSIANS;

% Calculate data cost per cluster
Dc = zeros([sz(1:2) N_GAUSSIANS],'single');
[P,nlogl]=posterior(gmdist,img(:));
nlogP = -.3*log(P);
for ci=1:N_GAUSSIANS
    Dc(:,:,ci) = reshape(nlogP(:,ci),sz(1:2));
end


% smoothness term: 
% constant part
Sc = ones(k) - eye(k);
% spatialy varying part
% [Hc Vc] = gradient(imfilter(rgb2gray(im),fspecial('gauss',[3 3]),'symmetric'));
[Hc Vc] = SpatialCues(img);

time = tic();
gch = GraphCut('open', Dc, 10*Sc, exp(-Vc*5), exp(-Hc*5));
[gch L] = GraphCut('expand',gch);
gch = GraphCut('close', gch);
elapsedtime = toc(time);
disp(['GraphCut completed in ' num2str(elapsedtime) ' seconds.'])

% show results
figure(GRAPHCUT_FIGURE)
% imshow(img/MAX_DIST);
% hold on;
% colormap gray
% PlotLabels(L);
labelimg = label2rgb(L);
imshow(labelimg);
title('Results of graphcut')

total_time = toc(total_time);
disp(['Finished. Total time = ' num2str(total_time) ' seconds.'])