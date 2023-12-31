clear
rlocaxis = 1.5*[-1 1 -1 1];
tlim = [0 3];

m = 2.7*10^-3;
r = 40;
I = 2/3*m*r^2;
g = 9.8*10^3;
c = 0; % need to measure
h = 0.1; %0.03; % need to measure
d = 0; %4*h; % need to measure
a_1 = c/(m + I/r^2);
b_0 = (g*m)/(m + I/r^2);

if c == 0
    num = pi/180*b_0*h^2*[1 1];
    den = [2, -4, 2];
    G = tf(num,den,h);
else
    num = pi/180*b_0*[a_1*h - exp(a_1*h) + 1, a_1*h*exp(a_1*h) - exp(a_1*h) + 1];
    den = a_1^2*[1, - exp(a_1*h) - 1, exp(a_1*h)];
    G = tf(num,den,h);
end

%% Plant with Delay
P_delay = tf(1,[1 zeros(1,d/h)],h);

G_P = G*P_delay;

figure(1)
subplot(2,5,1); rlocus(G_P); axis(rlocaxis); axis equal; shg
subplot(2,5,6); bode(G_P); shg

%% Lead Controller 1
omega_target = 1.8;
alpha = 50;
p = omega_target*sqrt(alpha);
z = omega_target/sqrt(alpha);

D_lead_1 = tf([1 z*h-1],[1 p*h-1],h);

L = G_P*D_lead_1;

figure(1)
subplot(2,5,2); rlocus(L); axis(rlocaxis); axis equal; shg
subplot(2,5,7); bode(L); shg

% %% Low Pass Filter 1
% omega_c = 10;
% F_lpf = tf(omega_c*h,[1 omega_c*h-1],h);
% 
% L = L*F_lpf;
% 
% figure(2)
% subplot(2,3,2); rlocus(L); axis(rlocaxis); axis equal; shg
% subplot(2,3,5); bode(L); shg

% %% Lag Controller
% omega_target = 0.1;
% alpha = 10;
% p = omega_target/sqrt(alpha);
% z = omega_target*sqrt(alpha);
% 
% D_lag_1 = tf([1 z*h-1],[1 p*h-1],h);
% 
% L = G_P*D_lead_1*D_lag_1;
% 
% figure(1)
% subplot(2,5,3); rlocus(L); axis(rlocaxis); axis equal; shg
% subplot(2,5,8); bode(L); shg

%% Adjust Gain
K = 1/abs(freqresp(L,omega_target))

poles = rlocus(L,K);
figure(1)
subplot(2,5,4); rlocus(L); hold on; 
plot(real(poles),imag(poles),'r*'); hold off; axis(rlocaxis); axis equal

%% Closed Loop 1
D_z = K*D_lead_1; %*F_lpf;
T = D_z*G_P/(1+D_z*G_P);
figure(1)
subplot(2,5,5); step(T,tlim); shg
subplot(2,5,9); bode(T); shg

S_u = D_z/(1+G_P*D_z);
subplot(2,5,10); step(S_u,tlim); shg

%% Difference Equation 1
syms z Theta(z) E(z)
[num, den] = tfdata(D_z);
eqn = simplify( poly2sym(cell2mat(num), z)*E(z) == poly2sym(cell2mat(den), z)*Theta(z) );
eqn_diff = vpa(iztrans(eqn))

%% Ad Hoc PID
PID = pid(0.04,0,0.032,0,h);
L_PID = G_P*PID;

figure(3)
subplot(2,1,1)
poles = rlocus(L_PID,
rlocus(L_PID); hold on; 
plot(real(poles),imag(poles),'r*'); hold off; axis(rlocaxis); axis equal

subplot(2,1,2)
T_PID = L_PID/(1+L_PID);
step(T_PID,tlim); shg

%% PID Difference Equation
syms z Theta(z) E(z) theta_k x_k
[num, den] = tfdata(PID);
eqn = simplify( poly2sym(cell2mat(num), z)*E(z) == poly2sym(cell2mat(den), z)*Theta(z) );
eqn_diff = vpa(iztrans(eqn))

