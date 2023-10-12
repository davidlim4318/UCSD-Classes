% MAE 144 Homework 1
% Problem 2a
% David Lim
% A16398479
% 10/11/23

clear
syms s

% I. Initialization
b = RR_poly([-2 2 -5 5],1);
a = RR_poly([-1 1 -3 3 -6 6],1);
f = RR_poly([-1 -1 -3 -3 -6 -6],1);

% II. Evaluate: Diophantine equation
[x,y] = RR_diophantine(a,b,f)
test = trim(a*x+b*y);
residual = norm(f-test)

%{
***garbage***
a_sym = expand((s+1)*(s-1)*(s+3)*(s-3)*(s+6)*(s-6));
b_sym = expand((s+2)*(s-2)*(s+5)*(s-5));
f_sym = (s+1)*(s+1)*(s+3)*(s+3)*(s+6)*(s+6);
a_coeffs = fliplr(double(coeffs(a_sym)));
b_coeffs = fliplr(double(coeffs(b_sym)));
f_coeffs = fliplr(double(coeffs(f_sym)));
a = RR_poly(a_coeffs);
b = RR_poly(b_coeffs);
f = RR_poly(f_coeffs);
%}