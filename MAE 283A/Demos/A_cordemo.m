% Demo for XCORR (correlation function) and DFT (Discrete Fourier Transform)
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

% NOTE: change the value of N (number of data points) here to see effects
N=100;
u=randn(N,1);
R_u=xcorr(u,'biased');
tau=-N+1:N-1;

figure(1)
plot(tau,R_u);
xlabel('\tau');ylabel('R_u(\tau)');
title(['Auto covariance for N = ' num2str(N) ]);

%%

P_u=fft(R_u);
disp('Computation of Phi_u directly on the basis of R_u: notice how Phi_u is not real valued!');
disp(P_u(1:10))

%%
% reorder the R_u via the variable 'index'
index=[N:2*N-1 2*N-1 1:N-1];
figure(2)
plot(R_u(index));
xlabel('index');ylabel('R_u(\tau)');
title(['Reordered (symmetric) auto covariance for FFT']);

%% 

P_u=fft(R_u(index));
disp('Computation of Phi_u on the basis of re-ordered R_u: notice how Phi_u(w) is now real valued!');
disp(P_u(1:10))

% computation of spectrum via periodogram
U=fft(u);
P_up=U.*conj(U)/N;

% creation of a frequency vector
Deltaf=2*pi/N;
w=[0:Deltaf:pi];

% plot results

% notice how we plot P_u (computed via correlation function) for every 2nd
% value, as it was based on 2N (symmetric) data points in R_u instead of the N
% data points of the actual u or the U=fft(u), allowing us to compare the
% spectrum estimated via correlation functions and estimated via the
% periodogram
%%
figure(3)
plot(w,real(P_u(1:2:N+1)))
xlabel('w [rad/s]');ylabel('real{P_u(w)}');
grid
legend('spectral estimate via correlation function')

disp('The estimated spectrum can also be compared to the periodogram...');

%%
figure(3)
plot(w,real(P_u(1:2:N+1)),w,P_up(1:N/2+1))
xlabel('w [rad/s]');ylabel('real{P_u(w)}');
grid
legend('spectral estimate via correlation function','spectral estimate via periodogram')

disp('redo same exercise with larger N and see the variance on the estimate of P_u(w)')

%%
N=10000;
u=randn(N,1);
R_u=xcorr(u,'biased');
tau=-N+1:N-1;

figure(4)
plot(tau,R_u);
xlabel('\tau');ylabel('R_u(\tau)');
title(['Auto covariance for N = ' num2str(N) ]);
shg;

% reorder the R_u via the variable 'index'
index=[N:2*N-1 2*N-1 1:N-1];
% Computation of Phi_u on the basis of re-ordered R_u
P_u=fft(R_u(index));

% computation via periodogram
U=fft(u);
P_up=U.*conj(U)/N;

% creation of a frequency vector
Deltaf=2*pi/N;
w=[0:Deltaf:pi];

% plot results
%%
figure(5)
plot(w,real(P_u(1:2:N+1)),w,P_up(1:N/2+1))
xlabel('w [rad/s]');ylabel('real{P_u(w)}');
grid
legend('spectral estimate via correlation function','spectral estimate via periodogram')

disp('Notice how Phi_u(w) remains very "noisy", however mean(Phi_u(w)) = ');
mean(real(P_u(1:N/2+1)))

%%

% now look at a filtered white noise
disp('Now we compute a filtered signal y = lsim(F,u) and compare Phi_y(w) with |F(e^{jw})|^2');
%%

[num,den]=butter(2,0.1);
F=tf(num,den,1);
y=lsim(F,u);
% compute autocorrelation
R_y=xcorr(y,'biased');
% reorder the R_y via the variable 'index'
index=[N:2*N-1 2*N-1 1:N-1];
% Computation of Phi_y on the basis of re-ordered R_y
P_y=fft(R_y(index));
% frequency vector
Deltaf=2*pi/N;
w=[0:Deltaf:pi];
% Compute Bode response of filter
[m,p]=bode(F,w);
m=squeeze(m);
p=squeeze(p);
% plot results
figure(6);
l=loglog(w,abs(P_y(1:2:N+1)),w,m.^2,'r');
set(l(2),'linewidth',2)
axis([0 pi 1e-3 1e1])
grid
legend('spectral estimate via correlation function','theoretical value')
xlabel('w [rad/s]');ylabel('real{P_y(w)}');

disp('Notice again how Phi_y(w) remains very "noisy", however mean(Phi_y(w)) = |F(e^{jw})|^2');
%%

disp('Now we create an estimate of F(e^{jw}) by division of Phi_yu(w) and Phi_u(w)');
% compute crosscorrelation
R_yu=xcorr(y,u,'biased');
% reorder the R_yu via the variable 'index'
index=[N:2*N-1 2*N-1 1:N-1];
% Computation of Phi_yu on the basis of re-ordered R_yu
P_yu=fft(R_yu(index));
% create frequency vector
Deltaf=2*pi/N;
w=[0:Deltaf:pi];
% plot results
figure(7)
subplot(2,1,1);
l=semilogx(w,20*log(abs(P_yu(1:2:N+1)./P_u(1:2:N+1))),w,20*log(m),'r');
set(l(2),'linewidth',2)
grid
legend('transfer function estimate','theoretical value')
axis([0 pi -60 20])
xlabel('w [rad/s]');ylabel('mag [dB]');
subplot(2,1,2);
l=semilogx(w,180/pi*angle(P_yu(1:2:N+1)./P_u(1:2:N+1)),w,p,'r');
set(l(2),'linewidth',2)
grid
axis([0 pi -200 200])
xlabel('w [rad/s]');ylabel('angle [deg]');
