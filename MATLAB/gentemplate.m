function [template] = gentemplate(distance, angle)
% "distance" is the distance from the camera.
% "angle"   is the rotation of the shoulders (0-90 degrees)

run_as_script = ~exist('distance','var');

if run_as_script
    clear all;close all;
    distance = 1.0;
    angle = 0;
    run_as_script = true;
end

%== Generate a reference template ==%
nrows = 50;
ncols = 50;
[i j] = meshgrid(1:nrows, 1:ncols);
ref = zeros(nrows, ncols, 'single');

% Draw the head
head_rad = 10;
row_center = round(nrows/2);
col_center = 20;
head_points = (((i-row_center)).^2 + ((j-col_center)*.8).^2) < head_rad^2;

% Draw the shoulders
shoulder_rad = 30;
row_center = round(nrows/2);
col_center = ncols+10;
shoulder_squeeze = (angle/90)*1.5 + 1;
shoulder_points = (((i-row_center)*shoulder_squeeze).^2 + (j-col_center).^2) < shoulder_rad^2;

ref = max(ref, shoulder_points);
ref = max(ref, head_points);

%== Scale it ==%
DIST_0 = 1.5;
distance = max(distance, .5);

template = imresize(ref, DIST_0/distance, 'nearest');

if run_as_script
    figure(1)
    imshow(template)
end