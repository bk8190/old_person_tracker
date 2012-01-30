clear all;close all;

for i = 1:10
    [image name] = LoadImage(i);
    
    image_segmented = Segment(image);
    
    figure(1)
    imshow(image_segmented)
end