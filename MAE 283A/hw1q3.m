%% MAE 283A, Homework 1, Question 3, by David Lim
clear

% Load data
load('noisy_sine_data (1).mat')

% Plot data
figure(1)
plot(t,u)
title("u(t)")
xlabel("t")
ylabel("u")

% Define data length and sample time
N = length(u);
Delta_t = 0.001;

% Plot FFT magnitude
figure(2)
U = fft(u);
U_ = abs(U);
plot(2*pi/(N*Delta_t)*(0:N-1),U_)
title("Magnitude of FFT")
xlabel("\omega (rad/s)")
ylabel("|FFT(U)|")