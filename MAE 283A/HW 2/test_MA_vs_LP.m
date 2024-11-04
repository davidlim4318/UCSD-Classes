n = 4;
num = 1/n * ones(1,n);
den = [1 zeros(1,n-1)];
G = tf(num,den,-1)
roots(num)
%% 
alpha = 0.2;
num2 = alpha;
den2 = [1 alpha-1];
H = tf(num2,den2,-1)
%%
clf
bode(G)
hold on
bode(H)
hold off
shg
