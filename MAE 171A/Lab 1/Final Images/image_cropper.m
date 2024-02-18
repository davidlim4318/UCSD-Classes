images = dir('*.png') ;
n = length(images) ;
I = imread(images(end-8).name) ;  % crop one to get rect 
[x, rect] = imcrop(I) ;
for i = 1:n
    I = imread(images(i).name) ;   % REad image 
    I = imcrop(I,rect) ;           % crop image 
    fullFileName = fullfile(images(i).folder, ['cropped_' images(i).name]);
    imwrite(I,fullFileName);   % Save image
end