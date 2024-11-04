%% MAE 283A: Homework 2, Question 3 by David Lim

%% 3.1: Parameter estimation

% Load data
clear
load('mass_spring_damper.mat')

% Define indices
t_i = 3;
N = length(t);
idx = (t_i:N);

% Define data matrices
PHI = [ u(idx-1) u(idx-2) -y(idx-1) -y(idx-2) ];
Y = y(idx);

% Compute LS estimate
theta_LS = PHI\Y

%% 3.2: Simulation

% Sampling time:
Delta_T = mean(diff(t));

% Define discrete-time transfer function model
num = theta_LS(1:2)';
den = [1 theta_LS(3:4)'];
G = tf(num,den,Delta_T);

% Simulate model with input data
y_sim = lsim(G,u,t);

% Plot results
figure(1)
h = plot(t,[y y_sim],'LineWidth',2);
xlabel('time (s)')
ylabel('position (cm)')
legend('measured','simulated')
set(gca,'FontSize',14)
title('Position vs. Time of a Mass-Spring-Damper System','FontWeight','Normal','FontSize',18)

%% 3.3: Physical parameter estimation

w = 0:0.1:pi/Delta_T;
[mag,phase] = bode(G,w);

gain_DC = mag(1);

w_res = w( mag == max(mag) );

k_hat = 1/(0.01*gain_DC)

m_hat = k_hat/w_res^2

d2c(G,'zoh')