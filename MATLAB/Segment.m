% function Segment(in_img)

% if ~exist('in_img','var')
    run_as_script = true
    clear all;close all;
    [in_img ~] = LoadData('images/',12);
% end

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

% Generate a histogram
notatmax = img( img<(MAX_DIST-.05) );
[counts bins] = hist(notatmax, HIST_NBINS);

% Normalize the histogram
hist_dx = MAX_DIST/HIST_NBINS
counts = counts/(sum(counts)*hist_dx); 

samples = imresize(img, .3, 'nearest');
% figure(1)
% imshow(samples/MAX_DIST)
% pause;
% samples = samples( samples<(MAX_DIST-.05) );
% [samplecounts samplebins] = hist(samples, HIST_NBINS);
% samplecounts = samplecounts/(sum(samplecounts)*hist_dx)

disp('Fitting distribution')
time = tic();
options = statset('MaxIter', 500, 'Display', 'final');
gmdist = gmdistribution.fit(samples(:), N_GAUSSIANS, 'CovType', 'diagonal', 'Regularize', .001, 'Options', options);
elapsedtime = toc(time);
disp(['EM completed in ' num2str(elapsedtime) ' seconds.'])


%== Find histogram statistics with peaks and valleys ==%

% plot(samplebins,samplecounts, 'r-')

% Approximate the PDF of the data
[f xi] = ksdensity(notatmax, 'npoints', 200, 'width', .04);

% Find peaks
[~, peaklocs] = findpeaks(f, 'MINPEAKDISTANCE', 7);

% Assume valleys are halfway between peaks
npeaks = length(peaklocs);
valleylocs = zeros(npeaks-1, 1, 'int32');
for ipeak = 1:npeaks-1
    valleylocs(ipeak) = .5*(peaklocs(ipeak) + peaklocs(ipeak+1));    
end

% Refine the valley locations using gradient decent until they reach either
% local minima or bump up against peak locations
for ival = 1:length(valleylocs)
    newloc = valleylocs(ival);
    
    % Try to move it to the left
    while newloc > peaklocs(ival) && f(newloc-1)<f(newloc)
        newloc = newloc-1;
    end

    % Try to move it to the right
    while newloc < peaklocs(ival+1) && f(newloc+1)<f(newloc)
        newloc = newloc+1;
    end
    
    valleylocs(ival) = newloc;
end

% Show the histogram, approximated PDF, and peak/valley locations
figure(HIST_FIGURE);
hold on
plot(bins,counts, 'k-')
plot(xi,f, 'g-')
plot(xi(peaklocs), f(peaklocs)+.01, 'k^', 'markerfacecolor',[0 1 0]);
plot(xi(valleylocs), f(valleylocs)-.01, 'k^', 'markerfacecolor',[1 0 0]);
title('Histogram (blue) and approx. PDF (black) with local extrema')

% Now, divide up the data and train gaussians to each peak.
indices = 1:length(xi);

mu    = zeros(npeaks, 1, 'single');
sigma = zeros(npeaks, 1, 'single');
p     = zeros(1, npeaks, 'single');

dx = xi(2)-xi(1)

for igroup = 1:npeaks
    % Get the data from this group (special cases for first and last)
    if igroup == 1
        group_indices = indices<=valleylocs(1);
    elseif igroup == npeaks
        group_indices = indices>valleylocs(end);
    else
        group_indices = indices>valleylocs(igroup-1) & indices<=valleylocs(igroup);
    end
    
    group_x = xi(group_indices);
    group_f = f(group_indices);
    
    % Numerical integration to find the probability of this group
    p(igroup) = sum(group_f * dx);
    
    norm_group_f = group_f/(p(igroup));
    
    % First moment is the mean
    mu(igroup) = sum(group_x .* norm_group_f .* dx);
    
    % Second central moment is the variance
    sigma(igroup) = sum( (norm_group_f-mu
    
    figure(8)
    clf
    hold on
    plot(group_x,norm_group_f)
    line([mu(igroup), mu(igroup)], [min(norm_group_f), max(norm_group_f)]);
    hold off
    
    pause
end


p

% figure(GMDIST_FIGURE)
% x = (1:.01:MAX_DIST)';
% p = pdf(gmdist,x);
% plot(x,p, 'k-');
% title('PDF of Gaussian model')

figure(IMG_FIGURE)
imshow(img/MAX_DIST);
title('Image')

% time = tic();
% figure(CLUSTERS_FIGURE)
% idx = cluster(gmdist, img(:));
% idx = reshape(idx, size(img,1), size(img,2));
% imshow(label2rgb(idx, 'lines'))
% title('Clusters detected from EM')
% elapsedtime = toc(time);
% disp(['Clustering completed in ' num2str(elapsedtime) ' seconds.'])

return


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