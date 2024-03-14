clear
rlocaxis = [-4 4 -30 30];
tlim = [0 5];

k2 = 0.003860;
m1 = 1.711e-05;
d1 = (1.299e-05 + 1.303e-05)/2;
m2 = 1.842e-05;
d2 = 0;

G = tf(k2, [m1*m2 (d1*m2 + d2*m1) (d1*d2 + k2*m1 + k2*m2) (d1*k2 + d2*k2) 0]);

figure(10)
clf
subplot(2,5,1)
rlocus(G)
axis(rlocaxis)

subplot(2,5,2)
bode(G)

%% Notch Controller
z = 21;
p = 10;
D1 = tf([1 0 z^2],conv([1 p],[1 p]));

L = G*D1;

figure(10)
subplot(2,5,3)
rlocus(L)
axis(rlocaxis)

subplot(2,5,4)
bode(L)

%% Derivative Controller
D2 = tf([1 0.01],1);

L = G*D2*D1;

figure(10)
subplot(2,5,5)
rlocus(L)
axis(rlocaxis)

subplot(2,5,6)
bode(L)

%% Adjust Gain
K = 1.07e-5;

L = G*D1*D2;

poles = rlocus(L,K);
figure(10)
subplot(2,5,7)
rlocus(L)
hold on
plot(real(poles),imag(poles),'r*')
hold off
axis(rlocaxis)

subplot(2,5,8)
bode(L)

%% Closed Loop 1
C = D1*D2*K;
T = C*G/(1+C*G);
figure(10)
subplot(2,5,9); step(T,tlim); shg
subplot(2,5,10); bode(T); shg

% S_u = D_z/(1+G*D1);
% subplot(2,5,9); step(S_u,tlim); shg
