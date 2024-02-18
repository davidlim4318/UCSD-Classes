% DICe result reader, N. Boechler (1/2023)
% put this in a folder with your DICe_solution_0*.txt files

clc;
clear all;
close all;

mydata=readmatrix('DICe_solution_19.txt');

strain_yy=mydata(:,12);
xpos=mydata(:,2);
ypos=mydata(:,3);

figure(01)
scatter(xpos,ypos,[],strain_yy,'filled')
colorbar
xlabel('x')
ylabel('y')
set(gca,'fontsize',20,'linewidth',2)