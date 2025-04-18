%% DICeSolnReader.m
% Modified from the original DICeSolnReader script provided by N. Boechler.
% Reads each DICe .txt file, crops the data, and calculates the average
% strain.
% Place script inside folder containing DICe .txt files

clc
clear
close all

%% Identify gage section in data

yMinPx = 161;
yMaxPx = 2072;

mmPerPx = 2 * 25.4 / (2095 - 161); % mm distance divided by pixel distance

mydata = readmatrix('DICe_solution_00.txt'); % Reads first file

yPos = mydata(:,3); % only get y-coordinates first

% centroidYPx = sum(yPos)/length(yPos); % get y-coordinate of centroid
% gageLengthPx = 50 / mmPerPx; % define gage length in pixels
% 
% yMinPx = round(centroidYPx - gageLengthPx/2); % define lower bound of gage length
% yMaxPx = round(centroidYPx + gageLengthPx/2); % define upper bound of gage length

idx = find(yPos >= yMinPx & yPos <= yMaxPx); % find indices of data within bounds


% plot to check
figure(6)
clf

mydata = readmatrix('DICe_solution_00.txt');
strainYY = mydata(idx,12);
xPos = mydata(idx,2);
yPos = mydata(idx,3);

scatter(xPos,yPos,[],strainYY,'filled')
colorbar
title("(b)")
xlabel('x (px)')
ylabel('y (px)')
axis equal
axis([min(xPos)-60 max(xPos)+60 min(yPos)-15 max(yPos)+15])
ax = gca;
ax.TitleHorizontalAlignment = 'left';
set(ax,'FontSize',16)

%% Calculate average strains of each frame

frames = 10; % number of frames
strainAvg = zeros(frames,1); % placehoder for data

for i = 1:frames
    if i-1 < 10 % read the frame
        mydata = readmatrix(['DICe_solution_0' int2str(i-1) '.txt']);
    else
        mydata = readmatrix(['DICe_solution_' int2str(i-1) '.txt']);
    end
    strainYY = mydata(idx,12); % get strain data within bounds
    strainAvg(i) = mean(strainYY); % calculate average strain
end

plot(strainAvg) % plot average strain of each frame

%% Save average strain data

save("AverageStrain","strainAvg");
