%TRAJECTORYTEST  Test script for TRAJECTORYGENERATOR.
%   
%   TRAJECTORYTEST tests the functionality of TRAJECTORYGENERATOR by
%   generating a trajectory with specific SE(3) matrices.
%
%   See also TRAJECTORYGENERATOR.

%   Written by David Lim for the MAE 204 Final Project in WI25.
%   Last modififed on 03/08/25.

clear
addpath /Users/davidlim/Documents/ModernRobotics/packages/MATLAB/mr;

ang_e_init = pi/2;  % initial angle of end-effector
ang_e_fin = pi;  % final angle of end-effector for grasping
ang_c_init = 0;  % initial angle of block (given)
ang_c_fin = -pi/2;  % final angle of block (given)

% SE(3) configurations
Tse_initial = [cos(ang_e_init) 0 sin(ang_e_init) 0;
               0 1 0 0;
               -sin(ang_e_init) 0 cos(ang_e_init) 0.5;
               0 0 0 1];
Tsc_initial = [cos(ang_c_init) -sin(ang_c_init) 0 1;
               sin(ang_c_init) cos(ang_c_init) 0 0;
               0 0 1 0.05;
               0 0 0 1];
Tsc_final = [cos(ang_c_fin) -sin(ang_c_fin) 0 0;
             sin(ang_c_fin) cos(ang_c_fin) 0 -1;
             0 0 1 0.05;
             0 0 0 1];
Tce_grasp = [cos(ang_e_fin) 0 sin(ang_e_fin) 0;
             0 1 0 0;
             -sin(ang_e_fin) 0 cos(ang_e_fin) 0;
             0 0 0 1];
Tce_standoff = [cos(ang_e_fin) 0 sin(ang_e_fin) 0;
             0 1 0 0;
             -sin(ang_e_fin) 0 cos(ang_e_fin) 0.1;
             0 0 0 1];
k = 1;

% generate trajectories
[trajectory,csv_list] = TrajectoryGenerator(Tse_initial,Tsc_initial,Tsc_final,Tce_grasp,Tce_standoff,k);