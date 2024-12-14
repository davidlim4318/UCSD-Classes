%% MAE 280A Homework 3

% David Lim
% A16398479
clear
clc

% Define A matrix
A = [-0.045  0.036    -32     -2; ...
       -0.4     -3   -0.3    250; ...
          0      0      0      1; ...
          0      0      1      0];

% Define B matrix
B = [  0  0.1; ...
     -30    0; ...
       0    0; ...
     -10    0];

%% Exercise 1.1

% Define B matrix only with input mu
B_mu = B(:,2)

% Compute controllability matrix and rank
C_AB_mu = [B_mu A*B_mu A^2*B_mu A^3*B_mu]
rank(C_AB_mu)

% Define B matrix only with input delta
B_delta = B(:,1)

% Compute controllability matrix and rank
C_AB_delta = [B_delta A*B_delta A^2*B_delta A^3*B_delta]
rank(C_AB_delta)

%% Exercise 1.2

% Define C matrix only with output q
C_q = [0 0 0 1]

% Compute observability matrix and rank
O_AC_alpha = [    C_q; ...
                C_q*A; ...
              C_q*A^2; ...
              C_q*A^3]
rank(O_AC_alpha)

% Define C matrix only with output alpha
C_alpha = [0 1 0 0]

% Compute observability matrix and rank
O_AC_alpha = [    C_alpha; ...
                C_alpha*A; ...
              C_alpha*A^2; ...
              C_alpha*A^3]
rank(O_AC_alpha)