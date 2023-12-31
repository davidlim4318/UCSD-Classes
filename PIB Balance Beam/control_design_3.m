clear

% Parameters
m = 2.7; % mass of ball, g
r = 40; % radius of ball, mm
I = 2/3*m*r^2; % moment of inertia of ball, g*mm^2
g = 9.8; % graviational acceleration, m/s^2

J = 225; % moment of inertia of beam, g*mm^2
tau_st = 9.4; % stall torque of motor, kg*cm
omega_nl = 52; % no-load speed of motor, rpm
V = 5; % source voltage, V

h = 0.1; %0.03; % need to measure
d = 0; %4*h; % need to measure

% Conversions
m = m*10^-3; % g -> kg;
I = I*10^-3; % g*mm^2 -> kg*mm^2
g = g*10^3; % m/s^2 -> mm/s^2
J = J*60*10^-3; % g*mm^2 -> kg*mm^2
tau_st = tau_st*10; % kg*cm -> kg*mm
omega_nl = omega_nl*2*pi/60; % rpm -> rad/s

% Constants
b_0 = (g*m)/(m + I/r^2);
c_1 = tau_st/(omega_nl*J);
d_0 = tau_st/(V*J);

G_c = tf(b_0*d_0,[1 c_1 0 0 0]);
%G_c = tf(d_0,[1 c_1 0]);
G_z = c2d(G_c,h,'zoh');

rlocaxis = 1.5*[-1 1 -1 1];

%% Plant with Delay
P_delay = tf(1,[1 zeros(1,d/h)],h);

G = G_z*P_delay;

figure(1)
subplot(2,5,1); rlocus(G); axis(rlocaxis); axis equal; shg
subplot(2,5,6); bode(G); shg

%% Lead Controller 1
omega_target = 1.8/1;
alpha = 40;
p = omega_target*sqrt(alpha);
%z = omega_target/sqrt(alpha);
z = 0;

D_lead_1 = tf([1 z*h-1],[1 p*h-1],h);

D1 = D_lead_1^3;

L1 = G*D1;
figure(1)
subplot(2,5,2); rlocus(L1); axis(rlocaxis); axis equal; shg
subplot(2,5,7); bode(L1); shg

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
K = 1/abs(freqresp(L1,omega_target))

poles = rlocus(L1,K);
figure(1)
subplot(2,5,4); rlocus(L1); hold on; 
plot(real(poles),imag(poles),'r*'); hold off; axis(rlocaxis); axis equal

%% Closed Loop 1
D = K*D1;
L = G*D;
T = L/(1+L);
if any(abs(pole(T)) > 1)
    error('Closed loop system is unstable!')
end
%%
tlim = [0 3];
figure(1)
subplot(2,5,5); step(T,tlim); shg
subplot(2,5,9); bode(T); shg

S_u = D/(1+G*D);
subplot(2,5,10); step(S_u,tlim); shg

%% Difference Equation 1
syms z Theta(z) E(z)
[num, den] = tfdata(D_z);
eqn = simplify( poly2sym(cell2mat(num), z)*E(z) == poly2sym(cell2mat(den), z)*Theta(z) );
eqn_diff = vpa(iztrans(eqn))

%% Ad Hoc PID
PID = pid(0.04,0,0.032,0,h);
L_PID = G*PID;

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

