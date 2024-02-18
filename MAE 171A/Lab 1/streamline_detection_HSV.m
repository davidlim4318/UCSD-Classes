%% Modified streamline_detection_example.m Program to use HSV
clc;
clear;
close all;

img_filename = 'airfoil_9053.png';

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
Href = 0.5;  % reference hue value (may need to be changed)
Hdist = 0.2;  % deviation allowed (may need to be changed)
Smin = 0.15;
Vmin = 0;

tempvec=zeros(1,3);
binary=zeros(size(HSV,1),size(HSV,2));

for i = 1:size(HSV,1)
    for j = 1:size(HSV,2)
        Htemp=HSV(i,j,1);
        Stemp=HSV(i,j,2);
        Vtemp=HSV(i,j,3);
        if abs(Href-Htemp) < Hdist && Stemp > Smin && Vtemp > Vmin % abs(Sref-Stemp) < Sdist
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
spl_smooth = smoothdata(spl,"movmean",100); % smooth the spline

load s_smooth.mat
x0 = 561;
x1 = 980;

% plotting
figure(5)
imshow(RGB); 
hold on
plot(x(x0:x1),s_smooth(x0:x1),'.c')
plot(x,spl_smooth,'.r')

%% Root Mean Squared Error From Airfoil Edge
pixels_per_square = 70.2;
mm_per_square = 26.6;
mm_per_pixel = mm_per_square/pixels_per_square;

rms_error = sqrt ( sum ( ( spl_smooth(x0:x1) - s_smooth(x0:x1) ).^2 ) ...
    / ( x1 - x0 + 1 ) ) * mm_per_pixel;
disp(rms_error)
