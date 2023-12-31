clear
rlocaxis = [-40 40 -40 40];
tlim = [0 10];

m = 2.7*10^-3;
r = 40;
I = 2/3*m*r^2;
g = 9.8*10^3;
c = 1; % need to measure
d = 0.12; % need to measure
h = 0.03; % need to measure
a_1 = c/(m + I/r^2);
b_0 = (g*m)/(m + I/r^2);

num = pi/180*b_0;
den = [1 a_1 0];
G = tf(num,den);

%% Plant with Delay
[num,den] = pade(d,2);
P_pade = tf(num,den);

G_P = G*P_pade;

figure(1)
subplot(2,1,1); rlocus(G_P); axis(rlocaxis); shg
subplot(2,1,2); bode(G_P); shg

%% Lead Controller 1
omega_target = 1;
alpha = 100;
p = omega_target*sqrt(alpha);
z = omega_target/sqrt(alpha);
D_lead_1 = tf([1 z],[1 p]);

L = G_P*D_lead_1;

figure(2)
subplot(2,3,1); rlocus(L); axis(rlocaxis); shg
subplot(2,3,4); bode(L); shg

%% Low Pass Filter 1
omega_c = 20;
F_lpf = tf(omega_c,[1 omega_c]);

L = L*F_lpf;

figure(2)
subplot(2,3,2); rlocus(L); axis(rlocaxis); shg
subplot(2,3,5); bode(L); shg

%% Adjust Gain
K = 1/abs(freqresp(L,omega_target))

poles = rlocus(L,K);
figure(2)
subplot(2,3,2); rlocus(L); hold on; plot(real(poles),imag(poles),'r*'); hold off; axis(rlocaxis); axis equal

%% Closed Loop 1
T = K*L/(1+K*L);
subplot(2,3,3); step(T,tlim); shg
subplot(2,3,6); bode(T); shg

%% Difference Equation 1
opts = c2dOptions('PrewarpFrequency',omega_target);
D_z = c2d(K*D_lead_1*F_lpf,h,opts)

syms z Theta(z) E(z) theta_k x_k
[num, den] = tfdata(D_z);
eqn = simplify( poly2sym(cell2mat(num), z)*E(z) == poly2sym(cell2mat(den), z)*Theta(z) );
eqn_diff = vpa(iztrans(eqn))

%% PD Controller 1
T_d = 100;

D_PD = tf([1 1/T_d],1);

L_PD = G_P*D_PD;

figure(3)
subplot(2,3,1); rlocus(L_PD); axis(rlocaxis); shg
subplot(2,3,4); bode(L_PD); shg

%% Low Pass Filter 1
omega_c = 20;
F_lpf = tf(omega_c,[1 omega_c]);

L_PD = L_PD*F_lpf;

figure(3)
subplot(2,3,2); rlocus(L_PD); axis(rlocaxis); shg
subplot(2,3,5); bode(L_PD); shg

%% Adjust Gain
omega_target_PD = 2;
K_p = 1/abs(freqresp(L_PD,omega_target_PD))
K_d = K_p*T_d

poles = rlocus(L_PD,K_p);
figure(3)
subplot(2,3,2); rlocus(L_PD); hold on; plot(real(poles),imag(poles),'r*'); hold off; axis(rlocaxis); axis equal

%% Closed Loop 2
T_PD = K_p*L_PD/(1+K_p*L_PD);
subplot(2,3,3); step(T_PD,tlim); shg
subplot(2,3,6); bode(T_PD); shg

%% Difference Equation 2
opts = c2dOptions('PrewarpFrequency',omega_target_PD);
D_PD_z = c2d(K_p*D_PD*F_lpf,h,opts)

syms z Theta(z) E(z) theta_k x_k
[num, den] = tfdata(D_PD_z);
eqn = simplify( poly2sym(cell2mat(num), z)*E(z) == poly2sym(cell2mat(den), z)*Theta(z) );
eqn_diff = vpa(iztrans(eqn))
