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

omega = logspace(-2,1,1000);
M = double(abs(L(1i*omega)));
phi = double(angle(L(1i*omega)));

jump = find(diff(phi)>0);
for i = 1:length(jump)
    phi = phi - 2*pi*[zeros(1,jump(i)) ones(1,length(phi)-jump(i))];
end

%%
figure(2)
subplot(2,1,1)
loglog(omega,M,linewidth=3)
title('Magnitude of G(s)*D(s)')
ylabel('Magnitude')
xlabel('Omega (rad/s)')
grid on
subplot(2,1,2)
semilogx(omega,180*phi/(2*pi),linewidth=3)
%axis([10^-2 10^1 -250 0])
title('Phase of G(s)*D(s)')
ylabel('Phase (deg)')
xlabel('Omega (rad/s)')
grid on
