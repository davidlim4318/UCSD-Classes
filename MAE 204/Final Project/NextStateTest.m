%NEXTSTATETEST  Test script for NEXTSTATE.
%   
%   NEXTSTATETEST simulates the KUKA youBot for T seconds with 
%   initial configuration theta_list, constant joint velocities
%   theta_dot_list, time step duration dt, and maximum joint/wheel velocity
%   theta_dot_max.
%   
%   NEXTSTATETEST saves theta_list of each time step as a
%   column in theta_array and writes the data to a csv file to be
%   visualized in CoppeliaSim Scene 6 associated with the book "Modern
%   Robotics: Mechanics, Planning, and Control," Kevin Lynch and Frank
%   Park, Cambridge University Press, 2017.
%
%   See also NEXTSTATE.

%   Written by David Lim for the MAE 204 Final Project in WI25.
%   Last modififed on 03/08/25.

clear

T = 1;  % total duration
theta_list = [0 0 0 0 0 0 0 0 0 0 0 0]';  % intial configuration
theta_dot_list = [pi/6 pi/6 pi/6 pi/6 pi/6 2*pi -2*pi 2*pi -2*pi]'; % joint/wheel velocities
dt = 0.01;  % time step
theta_dot_max = 2*pi;  % maximum velocity magnitude

N = T/dt+1;  % total steps
theta_array = zeros(13,N);  % array of all configurations
theta_array(1:12,1) = theta_list;

% main simulation loop
for i = 1:N-1
    theta_array(1:12,i+1) = (NextState(theta_array(1:12,i),theta_dot_list,dt,theta_dot_max))';
end

writematrix(theta_array','nextstate.csv')  % write matrix to csv file
