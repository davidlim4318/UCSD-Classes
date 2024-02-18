%%
figure(4)
G_tf = tf(0.1,[1 0.1],'InputDelay',d);
PID_tf = tf(K_p*T_D*[1 1/T_D 1/(T_D*T_I)],[1 0]);
L = PID_tf*G_tf;
bode(L)

%%
clear
a_0 = 0.1;
b_0 = 0.1;
d = 6;
syms s
%G(s) = b_0*exp(-d*s)/(s + a_0);
G(s) = b_0*exp(-d*s)/(s + a_0);

K_u = 3.32;
omega_u = 0.317;
T_u = 1/omega_u;

alpha = 0.6;
beta = 0.5;
gamma = 0.125;
K_p = alpha*K_u;
T_I = beta*T_u;
T_D = gamma*T_u;

D(s) = K_p*T_D*(s^2+s/T_D+1/(T_I*T_D))/s;

L(s) = G*D;

T(s) = simplify(expand(L/(1+L)));

%%
syms t
u_r(t) = 20 + 15*heaviside(t) + 10*heaviside(t-1*60) - 25*heaviside(t-4*60);
U_r(s) = laplace(u_r);

Y(s) = simplify(U_r*T);
y = ilaplace(Y);
figure(4)
fplot(y,[0,5*60])
grid on
xticks(0:60:300)
shg

% error("also, dont forget inital condition")