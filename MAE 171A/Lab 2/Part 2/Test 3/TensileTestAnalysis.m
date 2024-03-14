%% TensileTestAnalysis.m
% Analyzes force vs. displacement data and average strain data to determine
% elastic modulus, yield strength, and ultimate strength of material

%clc
clear
%close all

% read .csv file
M = readmatrix("test3.csv");

% isolate data
data = rmmissing(M(1:100,2:4));

% define cross-sectional area of sample (m^2)
A = 2.86 * 19 * 10^-6;

% define displacements (mm)
x = data(3:end,2);

% define forces (kN)
F = data(3:end,3);

figure(3)
plot(x,F,'b.','MarkerSize',10)
title("(a)")
xlabel("Displacement (mm)")
ylabel("Force (kN)")
legend('Data','Location','northeast')
ax = gca;
ax.TitleHorizontalAlignment = 'left';
set(ax,'FontSize',12)

timeWindow = 10; % window of time (s) to apply linear approximation

dispPerFrame = 2 / 60 * 10 * (0:1:timeWindow/10)'; % displacement in every frame in time window

%% Calculate elastic modulus

load('AverageStrain.mat') % get average strain data
strain = [0; strainAvg];

% starting frame of strain data
% frameStart = round( length(strain) - data(end,1) / 10 );
frameStart = 1;

% linear least-squares regression of strain data within time window (s1 is strain per mm displacement)
s = dispPerFrame \ ( strain( frameStart:(frameStart + timeWindow / 10) ) - strain(frameStart) );

% convert force (kN) corresponding stress (Pa)
sig = F / A * 1000;
% convert displacement (mm) to corresponding strain
eps = x * s;

n = timeWindow + 1;

% linear least-squares regression of force (kN) vs. displacement (mm) data, 
% then conversion to stress (Pa) vs. strain to obtain elastic modulus
E = eps(1:n)\sig(1:n);

% identify yield stress
yieldIdx = find(sig < E*(eps-0.002),1);
Sy = sig(yieldIdx);

% identify ultimate stress
Su = max(sig);
ultIdx = find(sig == Su);

figure(3)
clf
tiledlayout(1,2)
nexttile(1)
plot(0:length(strain)-1,strain(frameStart:end),'b.','MarkerSize',10)
hold on
plot(0:1:timeWindow/10,s*(0:1:timeWindow/10) * (2 / 60 * 10),'r-','LineWidth',2)
hold off
title("(c)")
xlabel("Frame Index")
ylabel("Average Strain")
legend('Data','Linear Least-Squares Approx.','Location','southeast',"FontSize",14)
ax = gca;
ax.TitleHorizontalAlignment = 'left';
set(ax,'FontSize',16)

nexttile(2)
plot(eps,sig,'b.','MarkerSize',10)
hold on
plot(eps(1:n),E*eps(1:n),'r-','LineWidth',2)
%plot(eps(1:yieldIdx)+0.002,E*(eps(1:yieldIdx)),'m--','LineWidth',2)
%plot(eps(yieldIdx),sig(yieldIdx),'m*','MarkerSize',10,'LineWidth',2)
plot(eps(ultIdx),sig(ultIdx),'mo','MarkerSize',10,'LineWidth',2)
hold off
title("(d)")
xlabel("Average Strain")
ylabel("Stress (Pa)")
%legend('Data','Linear Least-Squares Approx.','2% Strain Offset Line','Yield Point','Ultimate Stress','Location','southeast')
legend('Data','Linear Least-Squares Approx.','Ultimate Stress','Location','best',"FontSize",14)
ax = gca;
ax.TitleHorizontalAlignment = 'left';
set(ax,'FontSize',16)
