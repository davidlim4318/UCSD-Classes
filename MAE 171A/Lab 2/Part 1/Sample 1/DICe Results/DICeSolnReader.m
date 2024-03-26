clc
clear
close all

mmPerPx = 113.8 / 4262;

mydata = readmatrix('DICe_solution_00.txt');

yPos = mydata(:,3);

centroidYPx = sum(yPos)/length(yPos);
gageLengthPx = 50 / mmPerPx;

yMinPx = round(centroidYPx - gageLengthPx/2);
yMaxPx = round(centroidYPx + gageLengthPx/2);

idx = find(yPos >= yMinPx & yPos <= yMaxPx);


figure(6)
clf
tiledlayout(1,2)

mydata = readmatrix('DICe_solution_11.txt');
strainYY = mydata(idx,12);
xPos = mydata(idx,2);
yPos = mydata(idx,3);

nexttile(1)
scatter(xPos,yPos,[],strainYY,'filled')
colorbar
title("(a)")
xlabel('x (px)')
ylabel('y (px)')
axis equal
axis([min(xPos)-60 max(xPos)+60 min(yPos)-15 max(yPos)+15])
ax = gca;
ax.TitleHorizontalAlignment = 'left';
set(ax,'FontSize',16)

mydata = readmatrix('DICe_solution_21.txt');
strainYY = mydata(idx,12);
xPos = mydata(idx,2);
yPos = mydata(idx,3);

nexttile(2)
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

%%

frames = 47;
strainAvg = zeros(frames,1);

for i = 1:frames
    if i-1 < 10
        mydata = readmatrix(['DICe_solution_0' int2str(i-1) '.txt']);
    else
        mydata = readmatrix(['DICe_solution_' int2str(i-1) '.txt']);
    end
    strainYY = mydata(idx,12);
    strainAvg(i) = mean(strainYY);
end

plot(strainAvg)

%%
save("AverageStrainSample1","strainAvg");
