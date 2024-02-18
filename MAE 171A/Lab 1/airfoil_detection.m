%% airfoil_detection.m
clc;
clear;
close all;

img_filename = 'airfoil_9054.png';

RGB = imread(img_filename);
figure(1)
imshow(RGB);

pause  % press any key to advance to next step

%% RGB to HSV
HSV = rgb2hsv(RGB);  % maps rgb values to hsv
figure(2)
imshow(HSV)  % image will look weird because this function assumes rgb
% use tooltips to check hsv values of specific pixels

pause

%% Binary Filter for HSV
Href = 0.11;  % reference hue value (may need to be changed)
Hdist = 0.041;  % deviation allowed (may need to be changed)
Vmax = 0.2;  % sets maximum pixel brightness, in my case not needed

tempvec=zeros(1,3);
binary=zeros(size(HSV,1),size(HSV,2));

for i = 1:size(HSV,1)
    for j = 1:size(HSV,2)
        Htemp=HSV(i,j,1);
        Vtemp=HSV(i,j,3);
        if abs(Href-Htemp) < Hdist && Vtemp < Vmax
            binary(i,j)=1;
        end
    end
end

figure(3)
imshow(binary);  % shows the filtered result

pause

%% Crop Single Streamline
figure(3)
imshow(binary);
ROI = drawfreehand;  % click and drag around the desired streamline

pause  % press any key to confirm crop region

mask = ROI.createMask();
maskedbinary = binary .* mask;  % only keeps values within crop region
figure(4)
imshow(maskedbinary);

pause

%% Fit Spline on Streamline (mostly the same code as the original)
[row,col] = find(maskedbinary); % find the rows and columns that are = 1

[C,ia,ic]=unique(col); % find the unique elements along x (columns)
unqrow=row(ia); % reduce the rows to only unique elements
unqcol=col(ia); % reduce the columns to only unique elements

x = 1:length(maskedbinary(1,:)); % define a continuous x vector

spl = spline(unqcol,unqrow,x); % fit a spline
s_smooth = smoothdata(spl,"movmean",100); % smooth the spline

% plotting
figure(5)
imshow(RGB); 
hold on
plot(x,s_smooth,'.r')
