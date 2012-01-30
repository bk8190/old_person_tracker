function [mat, name] = LoadData(directory, choice)

% prefix = 'Needles/'
% suffix = '.JPG';
% filenames=...
% {'Needle_Single_DOF'
% 'Needle_On_Skin_Tissue'
% 'Needle_In_Skin_Tissue'
% 'Needle_On_Fat_Tissue'
% };


% directory = 'images/';

filenames = ls(directory);
filenames(1,:) = []; % Remove the first two entries (. and ..)
filenames(1,:) = [];
filenames = cellstr(filenames);


if ~exist('choice','var')
    choice = menu('Choose an image',filenames);
end

name = filenames{choice};

% % OVERRIDE:
% name = 'img_10'
mat = csvread([directory, name]);