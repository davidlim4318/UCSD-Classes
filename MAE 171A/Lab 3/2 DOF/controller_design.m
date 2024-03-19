clear
%rlocaxis = [-50 0 -20 20];
tlim = [0 5];

k2 = 2.710e-3;
m1 = 1.964e-6;
d1 = 1.630e-5;
m2 = 6.076e-6;
d2 = 1.657e-6;

% G = tf(k2, [m1*m2 (d1*m2 + d2*m1) (d1*d2 + k2*m1 + k2*m2) (d1*k2 + d2*k2) 0]);
G = 4000*tf( [1 2*0.02*3 3^2] , conv( conv([1 0],[1 2*0.03*4 4^2]) , [1 0.1] ) );
% G = 350000000*tf( [1 2*0.02*3 3^2] , conv ( conv( conv([1 0],[1 2*0.03*4 4^2]) , [1 2*0.05*45 45^2] ) , [1 2]) );

figure(10)
clf
subplot(2,5,1)
rlocus(G)
%axis(rlocaxis)
axis equal

subplot(2,5,2)
bode(G)

%{
%% Notch Controller
z = 4;
p = 100;
D1 = tf([1 0 z^2],conv([1 p],[1 p]));

L = G*D1;

figure(10)
subplot(2,5,3)
rlocus(L)
%axis(rlocaxis)
axis equal

subplot(2,5,4)
bode(L)
%}
D1 = 1

%% Derivative Controller
D2 = tf([1 10],1);

L = G*D2*D1;

figure(10)
subplot(2,5,5)
rlocus(L)
%axis(rlocaxis)
axis equal

subplot(2,5,6)
bode(L)

%% Adjust Gain
K = 0.02;

L = G*D1*D2;

poles = rlocus(L,K);
figure(10)
subplot(2,5,7)
rlocus(L)
hold on
plot(real(poles),imag(poles),'r*')
hold off
%axis(rlocaxis)
axis equal

subplot(2,5,8)
bode(L*K)

% Closed Loop 1
C = D1*D2*K;
T = C*G/(1+C*G);
figure(10)
subplot(2,5,9); step(T,tlim); shg
subplot(2,5,10); bode(T); shg

% S_u = D_z/(1+G*D1);
% subplot(2,5,9); step(S_u,tlim); shg
