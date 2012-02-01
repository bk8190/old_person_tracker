function Segment(in_img)

if ~exist('in_img','var')
    run_as_script = true
    clear all;close all;
    [in_img ~] = LoadData('images/',12);
end

total_time = tic();

img = imresize(in_img, .3, 'nearest');

MAX_DIST = 7;
HIST_NBINS = 80;

IMG_FIGURE = 1;
HIST_FIGURE = 2;
GAUS_APPROX_FIGURE = 6;
GMDIST_FIGURE = 3;
CLUSTERS_FIGURE = 4;
GRAPHCUT_FIGURE = 5;
GRAPHCUT_SPATIAL_FIGURE = 6;
CORR_FIGURE = 7;
CORR_FIGURE_2 = 8;

PDF_APPROX_RESOLUTION = .03;
PEAKS_MIN_DISTANCE = 5;
SIGMA_FACTOR = 1/(4^2);
GC_DATA_WEIGHT = .6;

% Remove NAN values
batman = isnan(img);
img(batman) = MAX_DIST;

% Median filter to remove noise
img = medfilt2(img, [4 4]);

sz = size(img);

% Generate a histogram
notatmax = img( img<(MAX_DIST-.05) );
[counts bins] = hist(notatmax, HIST_NBINS);

% Normalize the histogram
hist_dx = MAX_DIST/HIST_NBINS
counts = counts/(sum(counts)*hist_dx);

samples = imresize(img, .3, 'nearest');


%== Find histogram statistics with peaks and valleys ==%

% plot(samplebins,samplecounts, 'r-')

% Approximate the PDF of the data
[f xi] = ksdensity(notatmax, 'npoints', 200, 'width', PDF_APPROX_RESOLUTION);

% Find peaks
[~, peaklocs] = findpeaks(f, 'MINPEAKDISTANCE', PEAKS_MIN_DISTANCE);
k = length(peaklocs);

% Assume valleys are halfway between peaks
valleylocs = zeros(k-1, 1, 'int32');
for ipeak = 1:k-1
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
clf
hold on
plot(bins,counts, 'b-')
plot(xi,f, 'g-')
plot(xi(peaklocs), f(peaklocs)+.01, 'k^', 'markerfacecolor',[0 1 0]);
plot(xi(valleylocs), f(valleylocs)-.01, 'k^', 'markerfacecolor',[1 0 0]);
title('Histogram (blue) and approx. PDF (green) with local extrema labeled')
hold off

% Now, divide up the data and train gaussians to each peak.
indices = 1:length(xi);

mu    = zeros(k, 1, 'single');
sigma = zeros(1, 1, k, 'single');
p     = zeros(1, k, 'single');

dx = xi(2)-xi(1);
for igroup = 1:k
    % Get the data from this group (special cases for first and last)
    if igroup == 1
        group_indices = indices<=valleylocs(1);
    elseif igroup == k
        group_indices = indices>valleylocs(end);
    else
        group_indices = indices>valleylocs(igroup-1) & indices<=valleylocs(igroup);
    end
    
    group_x = xi(group_indices);
    group_f = f(group_indices);
    
    % Numerical integration to find the probability of this group
    p(igroup) = sum(group_f * dx);
    
    % Normalize so that the PDF of this group sums to 1
    norm_group_f = group_f/(p(igroup));
    
    % First moment is the mean
    mu(igroup) = sum(group_x .* norm_group_f .* dx);
    
    % Second central moment is the variance
    sigma(1,1,igroup) = sum( norm_group_f.*(group_x-mu(igroup)).^2 )*SIGMA_FACTOR;
    
    % Plot the current component along with its gaussian approximation
    %     Y = normpdf(xi, mu(igroup), sigma(igroup));
    
    %     figure(GAUS_APPROX_FIGURE)
%     if igroup == 1
%         clf
%     end
    %     hold on
    %     ylim([0 1])
    %     plot(group_x,group_f, 'b-')
    %     plot(xi,p(igroup)*Y, 'r-')
    %     line([mu(igroup), mu(igroup)], [min(group_f), max(group_f)]);
    %     title(['Group ', num2str(igroup), ', individual gaussian components'])
    %     hold off
    
end

gmdist = gmdistribution(mu, sigma, p)

figure(GMDIST_FIGURE)
clf
gaus_x = (0:.01:MAX_DIST)';
gaus_pdf = pdf(gmdist,gaus_x);
hold on
plot(gaus_x,gaus_pdf, 'k-');
plot(bins,counts, 'b-')
scatter(mu, mu*0, 'r*')
hold off
title('PDF of histogram (blue) and PDF of Gaussian model (black)')

% figure(IMG_FIGURE)
% imshow(img/MAX_DIST);
% title('Normalized Image')

time = tic();
figure(CLUSTERS_FIGURE)
idx = cluster(gmdist, img(:));
idx = reshape(idx, size(img,1), size(img,2));
imshow(label2rgb(idx, 'lines'))
title('Clusters detected from EM')
elapsedtime = toc(time);
disp(['Clustering completed in ' num2str(elapsedtime) ' seconds.'])

t1 = toc(total_time);
disp(['Finished. Total time = ' num2str(t1) ' seconds.'])

%== Now, find the posterior probability of each component for each pixel ==%

[PosteriorProbs,nlogl]=posterior(gmdist,img(:));
PosteriorProbImg = reshape(PosteriorProbs, [size(img,1), size(img,2), k]);
%== Perform template matching on each component ==%

for icomponent = 1:k
    thiscomponent = PosteriorProbImg(:,:,icomponent);
    
    % Generate the template
    template = gentemplate(mu(icomponent));
    
%     % Visualize the current component
%     imshow(thiscomponent/max(thiscomponent(:)))
%     hold on
%     imshow(template)
%     hold off
%     pause
    
    F = thiscomponent;
    T = template;
    
    % display frame and template
    figure(IMG_FIGURE)
%     subplot(121),imshow(F),title('Image');
    subplot(122)
    hold on
    imshow(F)
    imshow(T),title('Template');
    hold off
    
    % correlation matching
    [corrScore, boundingBox] = corrMatching(F,T,.6);
    
    % show results
    figure(CORR_FIGURE)
    imagesc(abs(corrScore)),axis image, axis off, colorbar,
    title('Corr Measurement Space')
    
    bY = [boundingBox(1),boundingBox(1)+boundingBox(3),boundingBox(1)+boundingBox(3),boundingBox(1),boundingBox(1)];
    bX = [boundingBox(2),boundingBox(2),boundingBox(2)+boundingBox(4),boundingBox(2)+boundingBox(4),boundingBox(2)];
    figure(IMG_FIGURE)
    subplot(121)
    imshow(F),line(bX,bY),title('Detected Area');
    pause
end

return

% Calculate data cost per cluster
Dc = zeros([sz(1:2) k],'single');
nlogP = -log(PosteriorProbs+.0001); %NOTE: Adding a small fraction to P to prevent log(0)
for ci=1:k
    Dc(:,:,ci) = GC_DATA_WEIGHT*reshape(nlogP(:,ci),sz(1:2));
end

% smoothness term:
% constant part
Sc = ones(k) - eye(k);
% spatialy varying part
[Hc Vc] = gradient(imfilter(img,fspecial('gauss',[3 3]),'symmetric'));
% [Hc Vc] = SpatialCues(img);
figure(GRAPHCUT_SPATIAL_FIGURE);
imshow(abs(max(Hc,Vc)));
title('Spatial gradient');

time = tic();
gch = GraphCut('open', Dc, 10*Sc, exp(-Vc*5), exp(-Hc*5));
[gch L] = GraphCut('expand',gch);
gch = GraphCut('close', gch);
elapsedtime = toc(time);
disp(['GraphCut completed in ' num2str(elapsedtime) ' seconds.'])

% show results
figure(GRAPHCUT_FIGURE)
clf
imshow(img/MAX_DIST);
hold on;
colormap gray
% PlotLabels(L);
labelimg = label2rgb(L);
imshow(labelimg);
title('Results of graphcut')
hold off

total_time = toc(total_time);
disp(['Finished. Total time = ' num2str(total_time) ' seconds.'])