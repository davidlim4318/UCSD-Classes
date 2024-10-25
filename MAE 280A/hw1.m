% MAE 280A Homework 1, Exercise 3.2
% David Lim
% A16398479
clear
clc

% Define matrix A
A = [2 6 2 8; ...
     2 7 3 9; ...
     1 5 3 1; ...
     1 2 0 8];

% Row reduce A to RREF, get the pivot column indices p
[B,p] = rref(A)

% Get the rank of A
k = length(p)

% Define X as the LI columns of A
X = A(:,p)

% Define Y as the matrix that solves the matrix equation X*Y' = A
Y = B(1:k,:)'   % Simply the non-zero rows of the RREF of A

% Check that X*Y' = A
isequal(X*Y',A)