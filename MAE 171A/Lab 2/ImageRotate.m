clc
clear
close all

myfile='DSC_6725.jpg';

angle = 271;
myimg=imrotate(imread(myfile),angle);
imshow(myimg);
%%
imwrite(myimg,'rotated_DSC_6725.jpg');