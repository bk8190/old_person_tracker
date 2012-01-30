function [numoffiles] = GetNumOfFiles(directory)
% 
% directory = 'images/';
filenames = ls(directory);
filenames(1,:) = []; % Remove the first two entries (. and ..)
filenames(1,:) = [];
filenames = cellstr(filenames);

numoffiles = length(filenames)

end