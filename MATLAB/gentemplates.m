function [templates] = gentemplates(ntemplates, distance)
angles = linspace(90, 0, ntemplates);

templates = cell(1,ntemplates);
for i = 1:ntemplates
    templates{i} = gentemplate(distance, angles(i));
end