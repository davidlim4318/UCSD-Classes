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


strainYY = mydata(idx,12);
xPos = mydata(idx,2);
yPos = mydata(idx,3);

figure(01)
scatter(xPos,yPos,[],strainYY,'filled')
colorbar
xlabel('x')
ylabel('y')
set(gca,'fontsize',20,'linewidth',2)
axis equal

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
