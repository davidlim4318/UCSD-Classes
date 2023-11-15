%%
clear
a_0 = 0.1;
b_0 = 0.1;
d = 6;

syms s
G(s) = b_0*exp(-d*s)/(s + a_0);

K_u = 3.2;
omega_u = 0.3;
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
y_r(t) = 35*heaviside(t) + 10*heaviside(t-1*60) - 25*heaviside(t-4*60);
Y_r(s) = laplace(y_r);

U(s) = Y_r*G;
u(t) = ilaplace(U);
figure(3)
fplot(u,[0,5*60])
shg

error("what's up with the scaling for this problem? also, don" + ...
    "t forget inital condition")