%%
clear
a_0 = 0.1;
b_0 = 0.1;
d = 6;

syms s
G(s) = b_0*exp(-d*s)/(s + a_0);

omega = logspace(-2,0,1000);
M = double(abs(G(1i*omega)));
phi = double(angle(G(1i*omega)));

jump = find(diff(phi)>0);
for i = 1:length(jump)
    phi = phi - 2*pi*[zeros(1,jump(i)) ones(1,length(phi)-jump(i))];
end

%%
figure(1)
subplot(2,1,1)
loglog(omega,M,linewidth=3)
title('Magnitude of G(s)')
ylabel('Magnitude')
xlabel('Omega (rad/s)')
grid on
subplot(2,1,2)
semilogx(omega,180*phi/(2*pi),linewidth=3)
title('Phase of G(s)')
ylabel('Phase (deg)')
xlabel('Omega (rad/s)')
grid on
