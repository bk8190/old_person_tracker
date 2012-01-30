clear all;close all;

directory = 'images/'
for i = 1:GetNumOfFiles(directory)
    [image name] = LoadData(directory, i);
    
    Segment(image);
%     image_segmented = Segment(image);
%     
%     figure(1)
%     imshow(image_segmented)
    pause
end