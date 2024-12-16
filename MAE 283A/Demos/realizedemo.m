% Demo for Realization 
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


systemchoice=[];
while isempty(systemchoice),
    systemchoice=input('Give choice for system: 1=model used in LSDEMO, 2=Garnier benchmark ');
    if systemchoice<0, systemchoice=[]; end
    if isstr(systemchoice), systemchoice=[]; end
    if imag(systemchoice)~=0, systemchoice=[]; end
    if max(size(systemchoice))~=1, systemchoice=[]; end
    if ((systemchoice~=1)&(systemchoice~=2)), systemchoice=[]; end
end    

clc
disp('Transfer function of orginal system')
if systemchoice==1.
    % use model to compare to lsdemo.m
    Ts=1;
    G0=tf([0.8 -0.5 0.6 0.5],[1 -0.1 0.6 0.1],Ts)
    % defined frequency vector for Bode plots
    w=logspace(-2,log10(pi/Ts),500);
    %noise level:
    lambda=0.1;
    % fixed axis for Bode plots
    m_axis=[1e-2 10 -10 10];
    p_axis=[1e-2 10 -200 400];
    % order of FIR model estimation
    kbar=14;
    % number of delays in model
    kdelay=0;
    [m,p]=bode(G0,w);
    figure(1)
    subplot(2,1,1)
    l=semilogx(w,20*log10(m(:)),'r');figure(gcf);
    set(l,'linewidth',1.5);
    title(['Amplitude Bode plot of discrete-time system'])
    ylabel('mag  [gain]')
    legend('G_0',3)
    axis(m_axis)
    grid
    subplot(2,1,2)
    l=semilogx(w,p(:),'r');figure(gcf);
    set(l,'linewidth',1.5);
    ylabel('phase  [deg]')
    xlabel('w  [rad/s]');
    axis(p_axis)
    grid
    pause
elseif systemchoice==2,
    % use model below for Rao-Garnier benchmark demo
    w1=2;w2=20;b1=0.1;b2=0.025;Tp=2;
    G=tf(-[1 -Tp]/Tp,conv([1/w1^2 2*b1/w1 1],[1/w2^2 2*b2/w2 1]))
    Ts=[];
    while isempty(Ts),
        Ts=input('Give sampling time of continuous-time system [0.1]: ');
        if Ts<0, Ts=[]; end
        if max(size(Ts))>1, Ts=[]; end
        if imag(Ts)~=0; Ts=[]; end
    end
    G0=c2d(G,Ts,'zoh')
    % defined frequency vector for Bode plots
    wd=logspace(-2,log10(pi/Ts),500);
    w=logspace(-2,log10(pi/0.01),500);
    % noise level
    lambda=2*Ts;
    % fixed axis for Bode plots
    m_axis=[1e-2 100 -70 30];
    p_axis=[1e-2 100 -600 50];
    % order of FIR model estimation
    kbar=floor(40*sqrt(0.1/Ts));  % the smaller, Ts, the larger kbar must be
    % number of delays in model
    kdelay=1;
    [md,pd]=bode(G0,wd);
    [m,p]=bode(G,w);
    figure(1)
    subplot(2,1,1)
    l=semilogx(wd,20*log10(md(:)),'b',w,20*log10(m(:)),'r');figure(gcf);
    set(l,'linewidth',1.5);
    title(['Amplitude Bode plot of continuous-time system  and ZOH discrete-time equivalent'])
    ylabel('mag  [gain]')
    legend('G_{disc}','G_{cont}',3)
    axis(m_axis)
    grid
    subplot(2,1,2)
    l=semilogx(wd,pd(:),'b',w,p(:),'r');figure(gcf);
    set(l,'linewidth',1.5);
    ylabel('phase  [deg]')
    xlabel('w  [rad/s]');
    axis(p_axis)
    grid
    pause
else
    disp('Wrong choice...')
    return;
end
%%
disp('Generating noise free data')
N=1000;
u=randn(N,1);
%u=sign(u);
y=lsim(G0,u);
figure(2)
subplot(2,1,1);
title('output y(t)')
plot(y);
subplot(2,1,2);
plot(u)
title('input y(t)')
xlabel('samples')
%%

disp('Estimating and plotting cross correlation function')
Ruu=xcorr(u,u,N/2);
Ryu=xcorr(y,u,N/2);
imp=impulse(G0,(kbar+9)*Ts);
figure(3)
l=plot(-49:(kbar+9),[zeros(49,1);imp],'b',-49:(kbar+9),Ryu(N/2-49+1:N/2+(kbar+9)+1)/N,'r',-49:(kbar+9),Ruu(N/2-49+1:N/2+(kbar+9)+1)/N,'m');
set(l,'linewidth',1.5);
xlabel('\tau');
legend('impulse of G_0','R_y_u(\tau)','R_{uu}(\tau)');
title('Estimated cross correlation function compared to impulse response')
%%

disp('Although correlation functions look "noisy", they are pretty accurate')
Rhat=lsim(G0,Ruu);
figure(4)
l=plot(-49:(kbar+9),Rhat(N/2-49+1:N/2+(kbar+9)+1)/N,-49:(kbar+9),Ryu(N/2-49+1:N/2+(kbar+9)+1)/N,'r');
set(l,'linewidth',1.5);
xlabel('\tau');
legend('R_{yu,sim}(\tau)','R_y_u(\tau)');
title('Simulated cross correlation R_{yu,sim}(\tau) = G_0(q) R_u(\tau)')
disp('This means we need to take Ruu(tau) into account when estimating impulse respone coefficients,')
disp('which can be done with an FIR model when using white noise input!')
%%


disp(['Estimating and plotting coefficients of FIR model of order ' num2str(kbar-1) ' with ' num2str(kdelay) ' step(s) of delay'])
% code beow is the same as
%RU=toeplitz(Ruu(N/2+1:N/2+kbar));
%fYU=Rhat(N/2+1:N/2+kbar);
%theta2=[RU]\[fYU];

% % first create (zero) regressor matrix
% PHI=zeros(N-kbar-kdelay+1,kbar-kdelay);
% % fill up the regressor matrix with past input data
% for k=0:kbar-kdelay-1,
%     PHI(:,k+1)=u(kbar-k:N-k-kdelay);
% end
% theta=PHI\y(kbar+kdelay:N);
% if kdelay>0,
%     theta=[zeros(kdelay,1);theta];
% end
% 

% estimation with ARX
TH=arx([y u],[0 kbar-kdelay kdelay]);
[A,B,C,D,F]=th2poly(TH);
theta2=B';

% FIR estimation with projection + LS ( without this the Garnier benchmark does not work...)
U=toeplitz([u(1) zeros(1,N-1)],u);
U1=U(1:kbar,1:end);U2=U(kbar+1:end,1:end);
%U2p=eye(N)-U2'*inv(U2*U2')*U2;
U2p=eye(N)-U2'*(U2'\eye(N));
U1m=[U1*U2p]';
theta=U1m\[y'*U2p]';


figure(5)
l=plot(-49:(kbar+9),[zeros(49,1);imp],'b',-49:(kbar+9),[zeros(49,1);theta2;zeros((kbar+10)-length(theta2),1)],'r',-49:(kbar+9),[zeros(49,1);theta;zeros((kbar+10)-length(theta),1)],'m');
set(l,'linewidth',1.5);
xlabel('\tau');
legend('impulse of G_0',['impulse response of estimated FIR of order ' num2str(kbar-1) ' with ' num2str(kdelay) ' delay(s)'],['impulse response of estimated FIR with projection of order ' num2str(kbar-1) ' with ' num2str(kdelay) ' delay(s)']);
title('Estimated FIR coefficients compared to impulse response')
disp('As you can see in Figure 5: perfectly reconstructed impulse response coefficients!')
%%

disp('Now performing realization algorithm by creating Hankel matrix, SVD and extract A,B,C,D')
H=hankel(theta(2:kbar/2),theta(kbar/2:kbar-1));
Hbar=hankel(theta(3:kbar/2+1),theta(kbar/2+1:kbar));
[U,S,V]=svd(H);
figure(6);
plot(diag(S),'*')
xlabel('# singular values')
ylabel('singular value')
order=[];
while isempty(order),
    order=input('Give order of model: ');
    if order<=0, order=[]; end
    if isstr(order), order=[]; end
    if imag(order)~=0, order=[]; end
    if max(size(order))~=1, order=[]; end
end
sqrtdiagS=sqrt(diag(S(1:order,1:order)));
H1=U(:,1:order)*diag(sqrtdiagS);H2=diag(sqrtdiagS)*V(:,1:order)';
D=theta(1)
C=H1(1,:)
B=H2(:,1)
A=diag(sqrtdiagS.\1)*U(:,1:order)'*Hbar*V(:,1:order)*diag(sqrtdiagS.\1)
Ghat=ss(A,B,C,D,Ts);
if systemchoice==2,Ghat=d2c(Ghat,'zoh');end
[mghat,pghat]=bode(Ghat,w);
% plotting Bode plot to compare
figure(7)
subplot(2,1,1)
l=semilogx(w,20*log10(mghat(:)),'g',w,20*log10(m(:)),'r');figure(gcf);
set(l,'linewidth',1.5);
title(['Amplitude Bode plot of estimated model'])
ylabel('mag  [gain]')
legend('G_{realize}','G_0',3)
axis(m_axis)
grid
subplot(2,1,2)
l=semilogx(w,pghat(:),'g',w,p(:),'r');figure(gcf);
set(l,'linewidth',1.5);
ylabel('phase  [deg]')
xlabel('w  [rad/s]');
axis(p_axis)
grid
%%


disp('Same excercise, but with noisy data with OE structure (white noise output noise)')
disp('Note that ARX estimation will not work in this case (see lsdemo.m)')
H0=tf([1],[1],1);
e=sqrt(lambda)*randn(N,1);
v=lsim(H0,e);
ynf=lsim(G0,u);
y=ynf+v;
figure(8)
subplot(2,1,1);
title('output y(t)')
plot(1:N,y,1:N,v,'r');
legend('input y','noise v')
subplot(2,1,2);
plot(u)
title('input y(t)')
xlabel('samples')
%%


disp('Estimating and plotting cross correlation function')
Ruu=xcorr(u,u,N/2);
Ryu=xcorr(y,u,N/2);
figure(9)
l=plot(-49:(kbar+9),[zeros(49,1);imp],'b',-49:(kbar+9),Ryu(N/2-49+1:N/2+(kbar+9)+1)/N,'r',-49:(kbar+9),Ruu(N/2-49+1:N/2+(kbar+9)+1)/N,'m');
set(l,'linewidth',1.5);
xlabel('\tau');
legend('impulse of G_0','R_y_u(\tau)','R_{uu}(\tau)');
title('Estimated cross correlation function compared to impulse response')
%%


disp('Even if you would simulate Ryu(tau) it remains noisy, as we have noisy data')
Rhat=lsim(G0,Ruu);
figure(10)
l=plot(-49:(kbar+9),Rhat(N/2-49+1:N/2+(kbar+9)+1)/N,-49:(kbar+9),Ryu(N/2-49+1:N/2+(kbar+9)+1)/N,'r');
set(l,'linewidth',1.5);
xlabel('\tau');
legend('R_{yu,sim}(\tau)','R_y_u(\tau)');
title('Simulated cross correlation R_{yu,sim}(\tau) = G_0(q) R_u(\tau)')
disp('This means we need to take Ruu(tau) into account when estimating impulse respone coefficients,')
disp('which can be done with an FIR model when using white noise input!')
%%


disp(['Estimating and plotting coefficients of FIR model of order ' num2str(kbar-1) ' with ' num2str(kdelay) ' step(s) of delay'])
% % first create (zero) regressor matrix
% PHI=zeros(N-kbar-kdelay+1,kbar-kdelay);
% % fill up the regressor matrix with past input data
% for k=0:kbar-kdelay-1,
%     PHI(:,k+1)=u(kbar-k:N-k-kdelay);
% end
% % now estimate FIR filter coefficients
% theta=PHI\y(kbar+kdelay:N);
% % add additional delays
% if kdelay>0,
%     theta=[zeros(kdelay,1);theta];
% end

% FIR estimation with projection + LS (Without this the Garnier benchmark does not work!)
theta=U1m\[y'*U2p]';

figure(11)
l=plot(-49:(kbar+9),[zeros(49,1);imp],'b',-49:(kbar+9),[zeros(49,1);theta;zeros((kbar+10)-length(theta),1)],'r');
set(l,'linewidth',1.5);
xlabel('\tau');
legend('impulse of G_0',['impulse response of estimated FIR of order ' num2str(kbar-1)]);
title('Estimated FIR coefficients compared to impulse response')
disp('As you can see in Figure 3: fairly well reconstrcuted impulse response coefficients!')
%%

disp('Now performing realization algorithm by creating Hankel matrix, SVD and extract A,B,C,D')
H=hankel(theta(2:kbar/2),theta(kbar/2:kbar-1));
Hbar=hankel(theta(3:kbar/2+1),theta(kbar/2+1:kbar));
[U,S,V]=svd(H);
figure(12);
plot(diag(S),'*')
xlabel('# singular values')
ylabel('singular value')
order=[];
while isempty(order),
    order=input('Give order of model: ');
    if order<=0, order=[]; end
    if isstr(order), order=[]; end
    if imag(order)~=0, order=[]; end
    if max(size(order))~=1, order=[]; end
end
sqrtdiagS=sqrt(diag(S(1:order,1:order)));
H1=U(:,1:order)*diag(sqrtdiagS);H2=diag(sqrtdiagS)*V(:,1:order)';
D=theta(1)
C=H1(1,:)
B=H2(:,1)
A=diag(sqrtdiagS.\1)*U(:,1:order)'*Hbar*V(:,1:order)*diag(sqrtdiagS.\1)
Ghat=ss(A,B,C,D,Ts);
if systemchoice==2,Ghat=d2c(Ghat,'zoh');end
[mghat,pghat]=bode(Ghat,w);
% plotting Bode plot to compare
figure(13)
subplot(2,1,1)
l=semilogx(w,20*log10(mghat(:)),'g',w,20*log10(m(:)),'r');figure(gcf);
set(l,'linewidth',1.5);
title(['Amplitude Bode plot of estimated model'])
ylabel('mag  [gain]')
legend('G_{realize}','G_0',3)
axis(m_axis)
grid
subplot(2,1,2)
l=semilogx(w,pghat(:),'g',w,p(:),'r');figure(gcf);
set(l,'linewidth',1.5);
ylabel('phase  [deg]')
xlabel('w  [rad/s]');
axis(p_axis)
grid
%%


disp('Now do the noisy data excercise a 200 times Monte Carlo style to get an idea of variance...')
disp('Press any key to start...');
%%
for mc=1:200,
    fprintf('%d,',mc)
    if round(mc/25)==mc/25, disp(' ');end
    % create new noise
    e=sqrt(lambda)*randn(N,1);
    v=lsim(H0,e);
    y=ynf+v;
    
%     % create (zero) regressor matrix
%     PHI=zeros(N-kbar-kdelay+1,kbar-kdelay);
%     % fill up the regressor matrix with past input data
%     for k=0:kbar-kdelay-1,
%         PHI(:,k+1)=u(kbar-k:N-k-kdelay);
%     end
%     % now estimate FIR filter coefficients
%     theta=PHI\y(kbar+kdelay:N);
%     % add additional delays
%     if kdelay>0,
%         theta=[zeros(kdelay,1);theta];
%     end
    
    % FIR estimation with projection + LS (yeah! - without this the Garnier benchmark does not work!)
    theta=U1m\[y'*U2p]';

    
    % create Hankel matrix
    H=hankel(theta(2:kbar/2),theta(kbar/2:kbar-1));
    Hbar=hankel(theta(3:kbar/2+1),theta(kbar/2+1:kbar));
    [U,S,V]=svd(H);
    % we keep the order the same
    sqrtdiagS=sqrt(diag(S(1:order,1:order)));
    H1=U(:,1:order)*diag(sqrtdiagS);H2=diag(sqrtdiagS)*V(:,1:order)';
    D=theta(1);
    C=H1(1,:);
    B=H2(:,1);
    A=diag(sqrtdiagS.\1)*U(:,1:order)'*Hbar*V(:,1:order)*diag(sqrtdiagS.\1);
    Ghat=ss(A,B,C,D,Ts);
    % compute Bode plot data and store it
    if systemchoice==2,Ghat=d2c(Ghat,'zoh');end
    [mghat,pghat]=bode(Ghat,w);
    mghat_mc(:,mc)=mghat(:);
    pghat_mc(:,mc)=pghat(:);
end    
disp('done!')

% plotting Bode plot to compare
figure(14)
subplot(2,1,1)
l=semilogx(w,20*log10(mghat_mc),'g',w,20*log10(m(:)),'r');figure(gcf);
set(l,'linewidth',1.5);
title(['Amplitude Bode plot of estimated model'])
ylabel('mag  [gain]')
axis(m_axis)
grid
subplot(2,1,2)
l=semilogx(w,pghat_mc,'g',w,p(:),'r');figure(gcf);
set(l,'linewidth',1.5);
ylabel('phase  [deg]')
xlabel('w  [rad/s]');
axis(p_axis)
grid

