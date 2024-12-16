% Demo for Approximate Modeling
% Matlab portion of demo for MAE283a (2024)
% Written by R.A. de Callafon, Dept. of MAE, UCSD <callafon@ucsd.edu>

% Legal Dogma
% * This software is part of the course MAE283a: "Parametric System Identification - 
%   theory and tools (Fall, 2024)", or the "MAE283A 2024 course".
% * Use this software to study and understand the main computational tools 
%   behind the system identification techniques used in the MAE283A course only.
% * Feel free to re-use some of the code for your final project in the MAE283a course
% * UNAUTHORIZED COPYING, REPRODUCTION, REPUBLISHING, UPLOADING, POSTING, TRANSMITTING 
%   OR DUPLICATING OF ANY OF THE CODE BELOW OTHER THAN FOR THE PURPOSE OF
%   THE MAE283A COURSE IS PROHIBITED

clc

% definition of model
disp('Data Generating System')
disp('----------------------');
disp('G0:');
G0=tf([0    8.5899  -25.9341   29.1481  -14.6230    2.9126   -0.0866],[1.0000   -3.2588    4.0656   -2.3557    0.5653   -0.0021   -0.0136],1)
disp('H0:');
H0=tf([1 -0.3],[1 -0.9],1)

% computation of Hankel singular values
disp('Hankel Singular Values of G0:');
disp('---------------------------- ');
kbar = 100;imp=impulse(G0,kbar);H=hankel(imp(2:kbar/2),imp(kbar/2:kbar-1));G=svd(H);G(1:6)'
% or use:
%[Gb,G]=balreal(ss(G0));
%G'
disp('Could possibly be approximated by 3rd order model...') 
n=3;

% definition of frequency vector
w=logspace(-3,pi,200);

figure(1);
set(gcf,'color','white')
subplot(2,1,1);
bodemag(G0,w)
legend('G_0')
axis([1e-3 pi 15 25])
title(['Figure ' num2str(gcf) ': amplitude Bode plot of G_0 and H_0'])
subplot(2,1,2);
bodemag(H0,w);
legend('H_0')
axis([1e-3 pi -20 20])
pause

disp('Generating noisy data...')
disp('N=50000;u=randn(N,1);e=randn(N,1);v=lsim(H0,e);y=lsim(G0,u)+v;');
N=50000;u=randn(N,1);e=randn(N,1);
v=lsim(H0,e);y=lsim(G0,u)+v;
pause

figure(5)
set(gcf,'color','white')
plot(1:N,[y u v]),legend('output','input','noise');
title(['Figure ' num2str(gcf) ': noisy I/0 data'])
pause

disp(['Estimating ARX model of order ' num2str(n) ' with 1 step time delay']);
disp(['TH=arx([y u],[' num2str(n) ' ' num2str(n) ' 1]);']);
TH=arx([y u],[n n 1]);
[a,b,c,d,f]=th2poly(TH);
Ghat=tf(b,a,1);
Hhat=tf([1 0 0],a,1);
pause

figure(6);
set(gcf,'color','white')
subplot(2,1,1);
bodemag(G0,Ghat,w)
legend('G_0','G_{ARX}')
axis([1e-3 pi 15 25])
title(['Figure ' num2str(gcf) ': amplitude Bode plot of ARX model based on noisy data'])
subplot(2,1,2);
bodemag(H0,Hhat,w);
legend('H_0','H_{ARX}')
axis([1e-3 pi -20 20])
pause

figure(7);
set(gcf,'color','white')
step(G0,Ghat)
a=axis;
axis([a(1) a(2) a(3) 15])
legend('G_0','G_{ARX}')
title(['Figure ' num2str(gcf) ': step response of ARX model based on noisy data'])
pause

disp('Now, "correct/high" order ARX model estimation');
disp('TH=arx([y u],[6 6 1]);');
TH=arx([y u],[6 6 1]);
[a,b,c,d,f]=th2poly(TH);
Ghat=tf(b,a,1);
Hhat=tf([1 zeros(1,5)],a,1);
pause

figure(8);
set(gcf,'color','white')
subplot(2,1,1);
bodemag(G0,Ghat,w)
legend('G_0','G_{ARX}')
axis([1e-3 pi 15 25])
title(['Figure ' num2str(gcf) ': amplitude Bode plot of "correct/high" order ARX model based on noisy data'])
subplot(2,1,2);
bodemag(H0,Hhat,w);
legend('H_0','H_{ARX}')
axis([1e-3 pi -20 20])
pause


figure(9);
set(gcf,'color','white')
step(G0,Ghat)
a=axis;
axis([a(1) a(2) a(3) 15])
legend('G_0','G_{ARX}')
title(['Figure ' num2str(gcf) ': step response of high order ARX model based on noisy data'])
pause

disp(['Estimating OE model of order ' num2str(n) ' with 1 step time delay']);
disp(['TH=oe([y u],[' num2str(n) ' ' num2str(n) ' 1]);']);
TH=oe([y u],[n n 1]);
[a,b,c,d,f]=th2poly(TH);
Ghat=tf(b,f,1);
Hhat=tf(1,1,1);
pause 

figure(10);
subplot(2,1,1);
bodemag(G0,Ghat,w)
legend('G_0','G_{OE}')
title(['Figure ' num2str(gcf) ': amplitude Bode plot of OE model based on noisy data'])
axis([1e-3 pi 15 25])
subplot(2,1,2);
bodemag(H0,Hhat,w);
legend('H_0','H_{OE}')
axis([1e-3 pi -20 20])
pause

figure(11);
set(gcf,'color','white')
step(G0,Ghat)
a=axis;
axis([a(1) a(2) a(3) 15])
legend('G_0','G_{OE}')
title(['Figure ' num2str(gcf) ': step response of OE model based on noisy data'])
