% Need image processing toolbox
clc;
clear;
close all;

img_filename = 'IMG_9054.png'; % image filename in same directory

RGB = imread(img_filename); % open image
figure(1)
imshow(RGB);

%%
rgbref=[9000 26000 29000]; % reference color
clrdist=14000; % color distance

tempvec=zeros(1,3); % temporary holding vector 
binary=zeros(size(RGB,1),size(RGB,2)); % blank binary matrix

for i = 1:size(RGB,1) % loop through all elements of cropped image
    for j = 1:size(RGB,2)
        tempvec(1)=RGB(i,j,1);
        tempvec(2)=RGB(i,j,2);
        tempvec(3)=RGB(i,j,3);
        if norm(tempvec-rgbref) < clrdist 
            % check if the color is within color distance
            binary(i,j)=1; % if so, set to 1
        end
    end
end

figure(2)
imshow(binary);

%% 
HSV = rgb2hsv(RGB);
figure(3)
imshow(HSV)


%%
Href = 0.48;
Hdist = 0.3;
Vmin = 0.04;

tempvec=zeros(1,3); % temporary holding vector 
binary=zeros(size(HSV,1),size(HSV,2)); % blank binary matrix

for i = 1:size(HSV,1) % loop through all elements of cropped image
    for j = 1:size(HSV,2)
        Htemp=HSV(i,j,1);
        %%Stemp=HSV(i,j,2);
        Vtemp=HSV(i,j,3);
        if abs(Href-Htemp) < Hdist % && Vtemp > Vmin
            % check if the color is within color distance
            binary(i,j)=1; % if so, set to 1
        end
    end
end

figure(2)
imshow(binary);

%%
% cropbinary=imcrop(binary,[1 130 size(binary,2) 350]); 
% crop again around streamline

[row,col] = find(cropbinary); % find the rows and columns that are = 1
x=[1:length(binary(1,:))]; % define a continuous x vector

[C,ia,ic]=unique(col); % find the unique elements along x (columns)
unqrow=row(ia); % reduce the rows to only unique elements
unqcol=col(ia); % reduce the columns to only unique elements

s = spline(unqcol,unqrow,x); % fit a spline
s2 = s+130; % compensate for 2nd crop
s_smooth=smooth(s2,100); % smooth the spline


% plotting
figure(001)
imshow(RGB); 
hold on
viscircles(centers,radii);
plot(centers(1),centers(2),'xr','MarkerSize',20,'LineWidth',3);
plot(x,s2,'or')
plot(x,s_smooth,'.b')

pause(2)

figure(002)
imshow(cropbinary)
hold on
plot(x,s,'or')

pause(2)

figure(003)
imshow(binary)
hold on
viscircles(centers,radii);
plot(centers(1),centers(2),'xr','MarkerSize',20,'LineWidth',3);
plot(x,s_smooth,'.b')


