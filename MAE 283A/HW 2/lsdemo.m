% Demo for Least Squares (ARX) estimation 
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
disp('Transfer function of orginal system')
G0=tf([0.8 -0.5 0.6 0.5],[1 -0.1 0.6 0.1],1)

pause
disp('Generating noise free data')
N=1024;
u=randn(N,1);
y=lsim(G0,u);
figure(1)
plot([u y]);
legend('input u','output y')
title('I/O signals)')
pause

disp('Estimating and plotting cross/auto correlation functions')
Ruu=xcorr(u,u,N/2);
Ryu=xcorr(y,u,N/2);
Ryy=xcorr(y,y,N/2);
figure(2)
l=plot(-49:49,Ruu(N/2-49+1:N/2+49+1)/N,-49:49,Ryu(N/2-49+1:N/2+49+1)/N,'r');
set(l,'linewidth',1.5);
xlabel('\tau');
legend('R_u(\tau)','R_y_u(\tau)');
title('Estimated cross/auto correlation functions')
pause

disp('Although correlation functions look "noisy", they are pretty accurate!')
Rhat=lsim(G0,Ruu);
figure(3)
l=plot(-49:49,Rhat(N/2-49+1:N/2+49+1)/N,-49:49,Ryu(N/2-49+1:N/2+49+1)/N,'r');
set(l,'linewidth',1.5);
xlabel('\tau');
legend('R_{yu,sim}(\tau)','R_y_u(\tau)');
title('Simulated cross correlation R_{yu,sim}(\tau) = G_0(q) R_u(\tau)')
pause

disp('Comparing cross correlation function with impulse response')
imp=impulse(G0,49);
figure(4)
l=plot(0:49,imp,0:49,Ryu(N/2+1:N/2+49+1)/N,'r');
set(l,'linewidth',1.5);
xlabel('\tau');
legend('g_0(\tau)','R_y_u(\tau)');
title('Comparison with impulse response')
pause

disp('Estimate model parameters via LS using correlation functions:')
RU=toeplitz(Ruu(N/2+1:N/2+1+3));
RY=toeplitz(Ryy(N/2+1:N/2+1+2));
RYU=toeplitz(-Ryu(N/2+1-1:N/2+1+2),-Ryu(N/2+1-1:-1:N/2+1-3));
fYU=Ryu(N/2+1:N/2+1+3);
fY=-Ryy(N/2+2:N/2+2+2);
theta=[RU RYU;RYU' RY]\[fYU;fY]
disp('Estimate model parameters via LS using direct regression of data:')
theta=[u(4:N) u(3:N-1) u(2:N-2) u(1:N-3) -y(3:N-1) -y(2:N-2) -y(1:N-3)]\y(4:N)
disp('Note the slight difference!')
disp('This is due to the fact that xcorr tries to use the full data length to compute correlation functions')
disp('while data regression truncates the data to form the estimated correlation functions (better!).')
pause

disp('Generating noisy data with ARX structure (filtered output noise)')
H0=tf([1],[1 -0.1 0.6 0.1],1);
% noise level:
lambda=1;
e=sqrt(lambda)*randn(N,1);
v=lsim(H0,e);
ynf=lsim(G0,u);
y=ynf+v;
figure(5)
plot([u y v]);
legend('input u','output y','noise v')
title('I/O signals)')
pause

disp('Estimating and plotting cross/auto correlation functions')
Ruu=xcorr(u,u,N/2);
Ryu=xcorr(y,u,N/2);
Ryy=xcorr(y,y,N/2);

figure(6)
l=plot(0:49,imp,0:49,Ryu(N/2+1:N/2+49+1)/N,'r');
set(l,'linewidth',1.5);
xlabel('\tau');
legend('g_0(\tau)','R_y_u(\tau)');
title('Comparison with impulse response')
pause


disp('Performing Spectral Analysis to estimate frequency response of G_0 and H_0')
% multiply with hanning window of size gamma
gamma=N/16;
Ruu_weighted = Ruu.*[zeros((N-gamma)/2,1);hanning(gamma+1);zeros((N-gamma)/2,1)];    
Ryu_weighted = Ryu.*[zeros((N-gamma)/2,1);hanning(gamma+1);zeros((N-gamma)/2,1)];    
Ryy_weighted = Ryy.*[zeros((N-gamma)/2,1);hanning(gamma+1);zeros((N-gamma)/2,1)];    

% make it suitable for fft
Suu=1/N*fft([Ruu_weighted(N/2+1:N);Ruu_weighted(1:N/2)]);
Syu=1/N*fft([Ryu_weighted(N/2+1:N);Ryu_weighted(1:N/2)]);
Syy=1/N*fft([Ryy_weighted(N/2+1:N);Ryy_weighted(1:N/2)]);

% spectral estimate
P=Suu(1:N/2+1).\Syu(1:N/2+1);
H=Syy(1:N/2+1)-Suu(1:N/2+1).\(abs(Syu(1:N/2+1)).^2);

figure(7)
w=linspace(0,pi,N/2+1);
[m,p]=bode(G0,w);
[mh,ph]=bode(H0,w);
l=loglog(w,abs(P),'go',w,m(:),'r',w,sqrt(abs(H)),'bo',w,(mh(:))*sqrt(lambda),'m');figure(gcf);
set(l,'linewidth',1.5);
title(['Amplitude Bode plot of SPA with Hanning window of width ' num2str(gamma) ])
ylabel('mag  [gain]')
xlabel('w  [rad/s]');
legend('G_s_p_a','G_0','sqrt(\Phi_v)','|H_0| sqrt(\lambda)',3)
axis([1e-3 10 1e-1 1e1])
pause


% disp('Estimate model parameters via LS using correlation functions:')
% RU=toeplitz(Ruu(N/2+1:N/2+1+3));
% RY=toeplitz(Ryy(N/2+1:N/2+1+2));
% RYU=toeplitz(-Ryu(N/2+1-1:N/2+1+2),-Ryu(N/2+1-1:-1:N/2+1-3));
% fYU=Ryu(N/2+1:N/2+1+3);
% fY=-Ryy(N/2+2:N/2+2+2);
% theta=[RU RYU;RYU' RY]\[fYU;fY]
disp('Estimate model parameters via LS using direct regression of data:')
theta=[u(4:N) u(3:N-1) u(2:N-2) u(1:N-3) -y(3:N-1) -y(2:N-2) -y(1:N-3)]\y(4:N)
disp('Estimated models:')
Ghat=tf(theta(1:4)',[1 theta(5:7)'],1)
Hhat=tf([1 zeros(1,3)],[1 theta(5:7)'],1)
pause

disp('Comparing Bode plots of models with real system')
figure(8)
[mghat,pghat]=bode(Ghat,w);
[mhhat,phat]=bode(Hhat,w);
l=loglog(w,mghat(:),'g-',w,m(:),'r',w,mhhat(:),'b-',w,mh(:),'m');figure(gcf);
set(l,'linewidth',1.5);
title(['Amplitude Bode plot of ARX model'])
ylabel('mag  [gain]')
xlabel('w  [rad/s]');
legend('G_a_r_x','G_0','H_a_r_x','H_0',3)
axis([1e-3 10 1e-1 1e1])
pause

disp('Generating noisy data with filter structure (band-pass filtered output noise)')
% this is a band pass noise
[numh1,denh1]=butter(2,0.5);
[numh2,denh2]=butter(2,0.1,'high');
H0=tf(conv(numh1,numh2),conv(denh1,denh2),1);
e=sqrt(lambda)*randn(N,1);
v=lsim(H0,e);
ynf=lsim(G0,u);
y=ynf+v;
figure(9)
plot([u y v]);
legend('input u','output y','noise v')
title('I/O signals)')
pause

disp('Estimating and plotting cross/auto correlation functions')
Ruu=xcorr(u,u,N/2);
Ryu=xcorr(y,u,N/2);
Ryy=xcorr(y,y,N/2);

figure(10)
l=plot(0:49,imp,0:49,Ryu(N/2+1:N/2+49+1)/N,'r');
set(l,'linewidth',1.5);
xlabel('\tau');
legend('g_0(\tau)','R_y_u(\tau)');
title('Comparison with impulse response')
pause


disp('Performing Spectral Analysis to estimate frequency response of G_0 and H_0')
% multiply with hanning window of size gamma
gamma=N/16;
Ruu_weighted = Ruu.*[zeros((N-gamma)/2,1);hanning(gamma+1);zeros((N-gamma)/2,1)];    
Ryu_weighted = Ryu.*[zeros((N-gamma)/2,1);hanning(gamma+1);zeros((N-gamma)/2,1)];    
Ryy_weighted = Ryy.*[zeros((N-gamma)/2,1);hanning(gamma+1);zeros((N-gamma)/2,1)];    

% make it suitable for fft
Suu=1/N*fft([Ruu_weighted(N/2+1:N);Ruu_weighted(1:N/2)]);
Syu=1/N*fft([Ryu_weighted(N/2+1:N);Ryu_weighted(1:N/2)]);
Syy=1/N*fft([Ryy_weighted(N/2+1:N);Ryy_weighted(1:N/2)]);

% spectral estimate
P=Suu(1:N/2+1).\Syu(1:N/2+1);
H=Syy(1:N/2+1)-Suu(1:N/2+1).\(abs(Syu(1:N/2+1)).^2);

figure(11)
[mh,ph]=bode(H0,w);
l=loglog(w,abs(P),'go',w,m(:),'r',w,sqrt(abs(H)),'bo',w,(mh(:))*sqrt(lambda),'m');figure(gcf);
set(l,'linewidth',1.5);
title(['Amplitude Bode plot of SPA with Hanning window of width ' num2str(gamma) ])
ylabel('mag  [gain]')
xlabel('w  [rad/s]');
legend('G_s_p_a','G_0','sqrt(\Phi_v)','|H_0| sqrt(\lambda)',3)
axis([1e-3 10 1e-1 1e1])
pause

% disp('Estimate model parameters via LS using correlation functions:')
% RU=toeplitz(Ruu(N/2+1:N/2+1+3));
% RY=toeplitz(Ryy(N/2+1:N/2+1+2));
% RYU=toeplitz(-Ryu(N/2+1-1:N/2+1+2),-Ryu(N/2+1-1:-1:N/2+1-3));
% fYU=Ryu(N/2+1:N/2+1+3);
% fY=-Ryy(N/2+2:N/2+2+2);
% theta=[RU RYU;RYU' RY]\[fYU;fY]
disp('Estimate model parameters via LS using direct regression of data:')
theta=[u(4:N) u(3:N-1) u(2:N-2) u(1:N-3) -y(3:N-1) -y(2:N-2) -y(1:N-3)]\y(4:N)
disp('Estimated models:')
Ghat=tf(theta(1:4)',[1 theta(5:7)'],1)
Hhat=tf([1 zeros(1,3)],[1 theta(5:7)'],1)
pause

disp('Comparing Bode plots of models with real system')
figure(12)
[mghat,pghat]=bode(Ghat,w);
[mhhat,phat]=bode(Hhat,w);
l=loglog(w,mghat(:),'g-',w,m(:),'r',w,mhhat(:),'b-',w,mh(:),'m');figure(gcf);
set(l,'linewidth',1.5);
%axis([1e-2 1e1 1e-2 1])
title(['Amplitude Bode plot of ARX model'])
ylabel('mag  [gain]')
xlabel('w  [rad/s]');
legend('G_a_r_x','G_0','H_a_r_x','H_0',3)
axis([1e-3 10 1e-1 1e1])
disp('Compare Fig. 12 with Fig. 8: conflict in trying to fit H_0 causes bias in LS estimate!')

