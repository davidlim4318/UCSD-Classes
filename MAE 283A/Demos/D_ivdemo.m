% Demo for Instrumental Variable (IV) estimation 
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
% low order system (same order as estimate)
G0=tf([0.8 -0.5 0.6 0.5],[1 -0.1 0.6 0.1],1)
% this is a band pass noise
[numh1,denh1]=butter(2,0.5);
[numh2,denh2]=butter(2,0.1,'high');
numh=conv(numh1,numh2);
denh=conv(denh1,denh2);
numh=numh/numh(1);
H0=tf(numh,denh,1)
%%

% number of models to estimate for Monte Carlo
mc=100;

% number of iterations in IV estimation
iter=5;

% noise level:
lambda=0.2;

if exist('ivdata.mat')==2,
    disp('Loading data from IVDATA.MAT...');
    load ivdata
    N=length(u);
else
    disp('Generating noisy data with filter structure (band-pass filtered output noise)')
    N=5000;
    u=randn(N,1);
    e=sqrt(lambda)*randn(N,1);
    v=lsim(H0,e);
    % save ivdata u v
end
ynf=lsim(G0,u);
y=ynf+v;


figure(1)
plot([u y v]);
legend('input u','output y','noise v')
title('I/O signals)')
%%

disp('Estimating and plotting cross/auto correlation functions')
imp=impulse(G0,49);
Ruu=xcorr(u,u,N/2);
Ryu=xcorr(y,u,N/2);
Ryy=xcorr(y,y,N/2);

figure(2)
l=plot(0:49,imp,0:49,Ryu(N/2+1:N/2+49+1)/N,'r');
set(l,'linewidth',1.5);
xlabel('\tau');
legend('g_0(\tau)','R_y_u(\tau)');
title('Comparison with impulse response')
%%


disp('Performing Spectral Analysis to estimate frequency response of G_0 and H_0')
% multiply with hanning window of size gamma
gamma=N/2^2;
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

figure(3)
w=linspace(0,pi,N/2+1);
[m,p]=bode(G0,w);
[mh,ph]=bode(H0,w);
subplot(2,1,1);
l=loglog(w,m(:),'r',w,abs(P),'go',w,m(:),'r');
set(l,'linewidth',1.5);
title(['Amplitude Bode plot of SPA of G_0 with Hanning window of width ' num2str(gamma) ])
ylabel('mag  [gain]')
xlabel('w  [rad/s]');
legend('G_0','G_s_p_a',3)
axis([1e-3 10 1e-1 1e1])
subplot(2,1,2);
l=loglog(w,(mh(:))*sqrt(lambda),'r',w,sqrt(abs(H)),'go',w,(mh(:))*sqrt(lambda),'r');
set(l,'linewidth',1.5);
title(['Amplitude Bode plot of SPA of H_0 with Hanning window of width ' num2str(gamma) ])
ylabel('mag  [gain]')
xlabel('w  [rad/s]');
legend('|H_0| sqrt(\lambda)','sqrt(\Phi_v)',3)
axis([1e-3 10 1e-1 1e1])
figure(gcf);
%%

clc
disp('Estimate model parameters via LS using direct regression of data:')
theta_LS=[u(4:N) u(3:N-1) u(2:N-2) u(1:N-3) -y(3:N-1) -y(2:N-2) -y(1:N-3)]\y(4:N)
disp('Estimated model:')
Ghat=tf(theta_LS(1:4)',[1 theta_LS(5:7)'],1)
Hhat=tf([1 zeros(1,3)],[1 theta_LS(5:7)'],1)
%%

disp(['Performing ' num2str(mc) ' Monte-Carlo LS estimations...']);
mghat = zeros(length(w),mc);
mhhat = zeros(length(w),mc);
for n=1:mc,
    e=sqrt(lambda)*randn(N,1);
    v=lsim(H0,e);
    y=ynf+v;
    theta_mc=[u(4:N) u(3:N-1) u(2:N-2) u(1:N-3) -y(3:N-1) -y(2:N-2) -y(1:N-3)]\y(4:N);
    Ghat=tf(theta_mc(1:4)',[1 theta_mc(5:7)'],1);
    Hhat=tf([1 zeros(1,3)],[1 theta_mc(5:7)'],1);
    [m_dummy,p_dummy]=bode(Ghat,w);mghat(:,n)=squeeze(m_dummy);
    [m_dummy,p_dummy]=bode(Hhat,w);mhhat(:,n)=squeeze(m_dummy);
end
    
disp('Comparing Bode plots of models with real system')
figure(4)
subplot(2,1,1);
l=loglog(w,m(:),'r',w,mghat,'g-',w,m(:),'r');
set(l,'linewidth',1.5);
title(['Amplitude Bode plot of G_0 and ARX (Linear Regression) models'])
ylabel('mag  [gain]')
xlabel('w  [rad/s]');
legend('G_0','G_a_r_x',3)
axis([1e-3 10 1e-1 1e1])
subplot(2,1,2);
l=loglog(w,mh(:),'r',w,mhhat,'g-',w,mh(:),'r');
set(l,'linewidth',1.5);
title(['Amplitude Bode plot of H_0 and ARX (Linear Regression) models'])
ylabel('mag  [gain]')
xlabel('w  [rad/s]');
legend('H_0','H_a_r_x',3)
axis([1e-3 10 1e-1 1e1])
figure(gcf);
disp('NOTE: Conflict in trying to fit H_0 causes bias in LS estimate of G_0!')
%%


clc
fprintf('%s','Performing simple IV iterations: ')

% initialize the IV (iteration)
Giv=Ghat;
THETAIV=[theta_LS zeros(length(theta_LS),iter-1)];
Giv_unstable=zeros(iter,1);

% IV estimation: create noise free `output' via simulation through LS estimated model
for k=1:iter-1,
    fprintf('%d,',k)
    
    [numiv,deniv]=tfdata(Giv,'v');
    % tag if model was unstable
    Giv_unstable(k)=max(abs(roots(deniv)))>1;

    % perform non-causal filtering (so instability of Giv does not matter)
    U=fft(u);
    [re,im]=nyquist(Giv,[0:2*pi/length(u):2*pi-2*pi/length(u)]);
    XI=(re(:)+i*im(:)).*U;xi=real(ifft(XI));

    % update parameter estimate using IV solution
    RN=[u(4:N) u(3:N-1) u(2:N-2) u(1:N-3) -xi(3:N-1) -xi(2:N-2) -xi(1:N-3)]'*[u(4:N) u(3:N-1) u(2:N-2) u(1:N-3) -y(3:N-1) -y(2:N-2) -y(1:N-3)];
    FN=[u(4:N) u(3:N-1) u(2:N-2) u(1:N-3) -xi(3:N-1) -xi(2:N-2) -xi(1:N-3)]'*y(4:N);
    theta_IV=RN\FN;
    THETAIV(:,k+1)=theta_IV;
    Giv=tf(theta_IV(1:4)',[1 theta_IV(5:7)'],1);
end
disp(' done');

theta_IV
disp('Estimated model:')
Giv
%%

figure(5)
index=find(Giv_unstable==1);
l=plot(1:iter,THETAIV','o-',index,THETAIV(:,index)','*');
set(l,'linewidth',1.5);
xlabel('iterations')
ylabel('parameter values');
title('Progress on parameter convergence for iterative IV estimation')
disp('Inspect progress on parameter convergence for iterative IV estimation')
%%


disp(['Performing ' num2str(mc) ' Monte-Carlo iterative IV estimations...']);
mgiv = zeros(length(w),mc);
for n=1:mc,
    e=sqrt(lambda)*randn(N,1);
    v=lsim(H0,e);
    y=ynf+v;
    theta_mc=[u(4:N) u(3:N-1) u(2:N-2) u(1:N-3) -y(3:N-1) -y(2:N-2) -y(1:N-3)]\y(4:N);
    Ghat=tf(theta_mc(1:4)',[1 theta_mc(5:7)'],1);
    Giv=Ghat;
    for k=1:iter-1,
        % perform non-causal filtering (so instability of Giv does not matter)
        U=fft(u);
        [re,im]=nyquist(Giv,[0:2*pi/length(u):2*pi-2*pi/length(u)]);
        XI=(re(:)+i*im(:)).*U;xi=real(ifft(XI));
                
        % update parameter estimate using IV solution
        RN=[u(4:N) u(3:N-1) u(2:N-2) u(1:N-3) -xi(3:N-1) -xi(2:N-2) -xi(1:N-3)]'*[u(4:N) u(3:N-1) u(2:N-2) u(1:N-3) -y(3:N-1) -y(2:N-2) -y(1:N-3)];
        FN=[u(4:N) u(3:N-1) u(2:N-2) u(1:N-3) -xi(3:N-1) -xi(2:N-2) -xi(1:N-3)]'*y(4:N);
        theta_mc=RN\FN;
        Giv=tf(theta_mc(1:4)',[1 theta_mc(5:7)'],1);
        [m_dummy,p_dummy]=bode(Giv,w);mgiv(:,n)=squeeze(m_dummy);
    end
end


disp('Comparing Bode plots of models with real system')
figure(6)
subplot(2,1,1);
l=loglog(w,m(:),'r',w,mghat(:,1),'g-',w,mgiv(:,1),'b',w,mghat,'g-',w,mgiv,'b',w,m(:),'r');
set(l,'linewidth',1.5);
title(['Amplitude Bode plot of G_0, ARX and IV models'])
ylabel('mag  [gain]')
xlabel('w  [rad/s]');
legend('G_0','G_a_r_x','G_i_v',3)
axis([1e-3 10 1e-1 1e1])
subplot(2,1,2);
l=loglog(w,mh(:),'r',w,mhhat(:,1),'g-',w,ones(length(w),1),'b',w,mhhat,'g-',w,ones(length(w),1),'b',w,mh(:),'r');
set(l,'linewidth',1.5);
title(['Amplitude Bode plot of H_0, ARX and IV models'])
ylabel('mag  [gain]')
xlabel('w  [rad/s]');
legend('H_0','H_a_r_x','H_i_v',3)
axis([1e-3 10 1e-1 1e1])
figure(gcf);
disp('NOTE: Improved estimate due to use of instrument uncorrelated with (non-white equation) noise')
%%


% comment out 'return' below to run iv4 from SysID toolbox
return
disp(['Performing ' num2str(mc) ' Monte-Carlo optimal IV estimations...']);
mgiv4 = zeros(length(w),mc);
for n=1:mc,
    e=sqrt(lambda)*randn(N,1);
    v=lsim(H0,e);
    y=ynf+v;
    TH=iv4([y u],[3 4 0]);
    Giv4=tf(TH.B,TH.A,1);
    [m_dummy,p_dummy]=bode(Giv4,w);mgiv4(:,n)=squeeze(m_dummy);
end

disp('Comparing Bode plots of models with real system')
figure(7)
subplot(2,1,1);
l=loglog(w,m(:),'r',w,mghat(:,1),'g-',w,mgiv(:,1),'b',w,mgiv4(:,1),'c',w,mghat,'g-',w,mgiv,'b',w,mgiv4,'c',w,m(:),'r');
set(l,'linewidth',1.5);
title(['Amplitude Bode plot of G_0, ARX and IV models'])
ylabel('mag  [gain]')
xlabel('w  [rad/s]');
legend('G_0','G_a_r_x','G_i_v','G_i_v4',3)
axis([1e-3 10 1e-1 1e1])
subplot(2,1,2);
l=loglog(w,mh(:),'r',w,mhhat(:,1),'g-',w,ones(length(w),1),'b',w,ones(length(w),1),'c',w,mhhat,'g-',w,ones(length(w),1),'b',w,ones(length(w),1),'c',w,mh(:),'r');
set(l,'linewidth',1.5);
title(['Amplitude Bode plot of H_0, ARX and IV models'])
ylabel('mag  [gain]')
xlabel('w  [rad/s]');
legend('H_0','H_a_r_x','H_i_v','H_i_v4',3)
axis([1e-3 10 1e-1 1e1])
figure(gcf);
disp('NOTE: Improved estimate due to use of "optimal" instrument uncorrelated with (non-white equation) noise')
%%


