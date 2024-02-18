% MAE 144 Homework 1
% Problem 2b
% David Lim
% A16398479
% 10/11/23

% I. Initialization
clear
evalc('hw1_prob2a');
c = RR_poly(-20,1);

% II. Evaluation: Make D(s) properclc
k = 0;
while x < y
    f = f*c;
    [x,y] = RR_diophantine(a,b,f);
    k = k + 1;
    if k > 100
        error("That's not right!")
    end
end

k
x
y
f
test = trim(a*x+b*y);
residual = norm(f-test)