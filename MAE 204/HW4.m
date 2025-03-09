%% Homework 4, Exercise 4.14

% David Lim
% A16398479
% 02/07/25
clear
% Add path to MR functions
addpath /Users/davidlim/Documents/ModernRobotics/packages/MATLAB/mr;
% Define fixed parameters
L0 = 4;
L1 = 3;
L2 = 2;
L3 = 1;
h = 0.1;
% Define variable parameters
theta1 = pi/2;
theta2 = 3;
theta3 = pi;
% Define e-e zero position configuration
M = [-1 0 0 0; 0 1 0 L0+L2; 0 0 -1 L1-L3; 0 0 0 1];
% Define screw axes in {s} frame
S1 = [0 0 1 L0 0 0]';
S2 = [0 0 0 0 1 0]';
S3 = [0 0 -1 -L0-L2 0 -h]';
% Compute new e-e configuration
T1 = expm(VecTose3(S1*theta1))*expm(VecTose3(S2*theta2))*expm(VecTose3(S3*theta3))*M
% Define screw axes in {b} frame
B1 = [0 0 -1 L2 0 0]';
B2 = [0 0 0 0 1 0]';
B3 = [0 0 1 0 0 h]';
% Compute new e-e configuration
T2 = M*expm(VecTose3(B1*theta1))*expm(VecTose3(B2*theta2))*expm(VecTose3(B3*theta3))