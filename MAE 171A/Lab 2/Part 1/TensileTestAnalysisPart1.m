%% TensileTestAnalysis.m
% Analyzes force vs. displacement data and average strain data to determine
% elastic modulus, yield strength, and ultimate strength of material

%clc
clear
%close all

% read .csv files
M1 = readmatrix("A04_4_sample1_1.csv");
M2 = readmatrix("A04_4_sample2.datatable.csv");
M3 = readmatrix("A04_4_sample3.datatable.csv");

% isolate data
data1 = rmmissing(M1(:,2:4));
data2 = rmmissing(M2(:,2:4));
data3 = rmmissing(M3(:,2:4));

% define measured cross-sectional area of each sample (m^2)
A1 = 2.87 * 12.35 * 10^-6;
A2 = 2.81 * 12.37 * 10^-6;
A3 = 2.86 * 12.44 * 10^-6;

% define displacements for each sample (mm)
x1 = data1(:,2);
x2 = data2(:,2);
x3 = data3(:,2);

% define forces for each sample (kN)
f1 = data1(:,3);
f2 = data2(:,3);
f3 = data3(:,3);

figure(1)
clf
tiledlayout(1,3)
nexttile(1)
plot(x1,f1)
title("Sample 1: Force vs. Displacement")
xlabel("Displacement (mm)")
ylabel("Force (kN)")

figure(2)
clf
tiledlayout(1,3)
nexttile(1)
plot(x2,f2)
title("Sample 2: Force vs. Displacement")
xlabel("Displacement (mm)")
ylabel("Force (kN)")

figure(3)
clf
tiledlayout(1,3)
nexttile(1)
plot(x3,f3)
title("Sample 3: Force vs. Displacement")
xlabel("Displacement (mm)")
ylabel("Force (kN)")

timeWindow = 30; % window of time (s) to apply linear approximation

dispPerFrame = 2 / 60 * 10 * (0:1:timeWindow/10)'; % displacement in every frame in time window

%% Calculate elastic modulus of sample 1

load('AverageStrainSample1.mat') % get sample 1 average strain data
strain1 = strainAvg;

% starting frame of strain data (based on duration of test and imaging time interval)
frameStart1 = round( length(strain1) - data1(end,1) / 10 );

% linear least-squares regression of strain data within time window (s1 is strain per mm displacement)
s1 = dispPerFrame \ ( strain1( frameStart1:(frameStart1 + timeWindow / 10) ) - strain1(frameStart1) );

% convert force (kN) corresponding stress (Pa)
sig1 = f1 / A1 * 1000;
% convert displacement (mm) to corresponding strain
eps1 = x1 * s1;

n = timeWindow + 1;

% linear least-squares regression of force (kN) vs. displacement (mm) data, 
% then conversion to stress (Pa) vs. strain to obtain elastic modulus
E1 = eps1(1:n)\sig1(1:n);

% identify yield stress
yieldIdx1 = find(sig1 < E1*(eps1-0.002),1);
Sy1 = sig1(yieldIdx1);

% identify ultimate stress
Su2 = max(sig1);
ultIdx1 = find(sig1 == Su2);

figure(1)
nexttile(2)
plot(strain1(frameStart1:end))
hold on
plot(0:1:timeWindow/10,s1*(0:1:timeWindow/10) * (2 / 60 * 10))
hold off
title("Sample 1: Strain in Each Frame")
xlabel("Frame Index")
ylabel("Average Strain")

nexttile(3)
plot(eps1,sig1)
hold on
plot(eps1(1:n),E1*eps1(1:n))
plot(eps1(1:yieldIdx1)+0.002,E1*(eps1(1:yieldIdx1)))
plot(eps1(yieldIdx1),sig1(yieldIdx1),'r*')
plot(eps1(ultIdx1),sig1(ultIdx1),'r*')
hold off
title("Sample 1: Stress vs. Strain")
xlabel("Average Strain")
ylabel("Stress (Pa)")

%% Calculate elastic modulus of sample 2

load('AverageStrainSample2.mat')
strain2 = strainAvg;

frameStart2 = round( length(strain2) - data2(end,1) / 10 );

s2 = dispPerFrame \ ( strain2( frameStart2:(frameStart2 + timeWindow / 10) ) - strain2(frameStart2) );

sig2 = f2 / A2 * 1000;
eps2 = x2 * s2;

n = timeWindow + 1;

E2 = eps2(1:n)\sig2(1:n);

yieldIdx2 = find(sig2 < E2*(eps2-0.002),1);
Sy1 = sig2(yieldIdx2);

Su2 = max(sig2);
ultIdx2 = find(sig2 == Su2);

figure(2)
nexttile(2)
plot(strain2(frameStart2:end))
hold on
plot(0:1:timeWindow/10,s2*(0:1:timeWindow/10) * (2 / 60 * 10))
hold off
title("Sample 2: Strain in Each Frame")
xlabel("Frame Index")
ylabel("Average Strain")

nexttile(3)
plot(eps2,sig2)
hold on
plot(eps2(1:n),E2*eps2(1:n))
plot(eps2(1:yieldIdx2)+0.002,E2*(eps2(1:yieldIdx2)))
plot(eps2(yieldIdx2),sig2(yieldIdx2),'r*')
plot(eps2(ultIdx2),sig2(ultIdx2),'r*')
hold off
title("Sample 2: Stress vs. Strain")
xlabel("Average Strain")
ylabel("Stress (Pa)")

%% Calculate elastic modulus of sample 3

timeOffset = 12;

load('AverageStrainSample3.mat')
strain3 = strainAvg;

frameStart3 = round( length(strain3) - data3(end,1) / 10 + timeOffset / 10);

s3 = dispPerFrame \ ( strain3( frameStart3:(frameStart3 + timeWindow / 10) ) - strain3(frameStart3) );

sig3 = f3 / A3 * 1000;
eps3 = x3 * s3;

n = timeWindow + 1;

E3 = ( eps3((1:n) + timeOffset) - eps3(timeOffset + 1) ) \ ( sig3((1:n) + timeOffset) - sig3(timeOffset + 1) );

figure(3)
nexttile(2)
plot(strain3(frameStart3:end))
hold on
plot(0:1:timeWindow/10,s3*(0:1:timeWindow/10) * (2 / 60 * 10))
hold off
title("Sample 3: Strain in Each Frame")
xlabel("Frame Index")
ylabel("Average Strain")

nexttile(3)
plot(eps3,sig3)
hold on
plot(eps3(1:n),E3*eps3(1:n))
plot(eps3(1:yieldIdx3)+0.002,E3*(eps3(1:yieldIdx3)))
plot(eps3(yieldIdx3),sig2(yieldIdx3),'r*')
plot(eps3(ultIdx3),sig2(ultIdx3),'r*')
hold off
title("Sample 3: Stress vs. Strain")
xlabel("Average Strain")
ylabel("Stress (Pa)")
