%% 
clc;
clear;
close all;

M1 = readmatrix("A04_4_sample1_1.csv");
M2 = readmatrix("A04_4_sample2.datatable.csv");
M3 = readmatrix("A04_4_sample3.datatable.csv");

data1 = rmmissing(M1(:,2:4));
data2 = rmmissing(M2(:,2:4));
data3 = rmmissing(M3(:,2:4));

%%
A1 = 2.87*12.35;
A2 = 2.81*12.37;
A3 = 2.86*12.44;

x1 = data1(:,2);
f1 = data1(:,3);
x2 = data2(:,2);
f2 = data2(:,3);
x3 = data3(:,2);
f3 = data3(:,3);

n = 21;

b1 = x1(1:n)\f1(1:n);

%%
figure(1)
hold on
plot(x1,f1)
plot(x1(1:n),b1*x1(1:n))