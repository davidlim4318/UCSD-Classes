% Batch image cropper, N. Boechler (1/2023)
% put this in a folder with your photograps (and only your photographs)
% create a subfolder called 'cropped'

clc
clear
close all

myfile='DSC_5747.jpg';

angle = 271;
myimg=imrotate(imread(myfile),angle);
imshow(myimg);
%%
window = [1500 1800 2300-1500 3900-1800];
cropimg=imcrop(myimg,window);
imshow(cropimg);

%% apply to all files in folder

images=dir('*.jpg');

for i=1:length(images)

    myimg=imrotate(imread(images(i).name),angle);

    cropimg=imcrop(myimg,window);
    %imshow(cropimg);

    fullFileName = fullfile(images(i).folder, ['cropped_' images(i).name]);

    imwrite(cropimg,fullFileName);

end