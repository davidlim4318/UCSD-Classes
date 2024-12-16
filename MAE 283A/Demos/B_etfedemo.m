% Demo for ETFE (Emperical Transfer Function Estimate) and SPA (Spectral Analysis)
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

% load or define model

a=[   1.866769310305872e-001    1.104413671089849e-001    4.881935213654112e-001    1.268613150230994e-002
    1.104413671089849e-001   -4.556323954698034e-001   -1.107246369303791e-001    2.971222010528244e-001
    4.881935213654111e-001   -1.107246369303790e-001    2.307865720126950e-001    2.476220665249365e-001
    1.268613150231036e-002    2.971222010528242e-001    2.476220665249363e-001    5.221540406508839e-001];

b=[  -4.325648115282207e-001
                         0
    1.253323064748307e-001
    2.876764203585489e-001];

c=[0    1.190915465642999e+000    1.189164201652103e+000   -3.763327659331765e-002];

d=0;

sys=ss(a,b,c,d,1);

disp('Show Bode plot of orginal system');
w=linspace(0,pi,500);
[m,p]=bode(sys,w);
subplot(2,1,1)
loglog(w,abs(m(:)),'r');figure(gcf);axis([1e-2 1e1 1e-2 1])
title('Bode plot of orginal system')
ylabel('mag  [gain]') 
subplot(2,1,2);
semilogx(w,p(:),'r');figure(gcf);axis([1e-2 1e1 -200 200])
ylabel('phase  [deg]');
xlabel('w  [rad/s]');
%%

disp('First ETFE with NOISE FREE data (100x)...');

disp('- ETFE with non-zero (pre)initial and non-zero end conditions and small N');
figure(1)
N=2^8;
P=[];
for k=1:100,
    u=randn(N+50,1);
    y=lsim(sys,u);
    u=u(51:N+50);
    y=y(51:N+50);
    if k==1,
        clf
        plot(1:length(u),u,1:length(y),y)
        title('I/O signals with non-zero pre-initial and non-zero end conditions, small N')
        legend('input','output')
        pause
    end
    Y=fft(y);U=fft(u);
    P=[P U(1:N/2+1).\Y(1:N/2+1)];
end;
% plot results
w=linspace(0,pi,N/2+1);
[m,p]=bode(sys,w);
subplot(2,1,1)
loglog(w,abs(P),'g.',w,abs(m(:)),'r');figure(gcf);axis([1e-2 1e1 1e-2 1])
title('100 ETFEs with non-zero (pre)initial and non-zero end conditions')
ylabel('mag  [gain]')
subplot(2,1,2);
semilogx(w,180/pi*angle(P),'g.',w,p(:),'r');figure(gcf);axis([1e-2 1e1 -200 200])
ylabel('phase  [deg]');
xlabel('w  [rad/s]');
%%

disp('- ETFE with non-zero (pre)initial and non-zero end conditions, but large N');
figure(2)
N=2^13;
P=[];
for k=1:100,
    u=randn(N+50,1);
    y=lsim(sys,u);
    u=u(51:N+50);
    y=y(51:N+50);
    if k==1,clf,plot(1:length(u),u,1:length(y),y),title('I/O signals with non-zero pre-initial and non-zero end conditions, large N'),legend('input','output'),pause;end
    Y=fft(y);U=fft(u);
    P=[P U(1:N/2+1).\Y(1:N/2+1)];
end;
% plot results
w=linspace(0,pi,N/2+1);
[m,p]=bode(sys,w);
subplot(2,1,1)
loglog(w,abs(P),'g.',w,abs(m(:)),'r');figure(gcf);axis([1e-2 1e1 1e-2 1])
title('100 ETFEs with non-zero pre-initial and non-zero end conditions, but large N')
ylabel('mag  [gain]')
subplot(2,1,2);
semilogx(w,180/pi*angle(P),'g.',w,p(:),'r');figure(gcf);axis([1e-2 1e1 -200 200])
ylabel('phase  [deg]');
xlabel('w  [rad/s]');
%%


disp('- ETFE with periodic applied input')
figure(3)
N=2^10;
P=[];
for k=1:100,
    u=randn(N,1);
    u=[u;u;u];
    y=lsim(sys,u);
    u=u(2*N+1:3*N);
    y=y(2*N+1:3*N);
    if k==1,clf,plot(1:length(u),u,1:length(y),y),title('I/O signals that are periodic (only one period shown)'),legend('input','output'),pause;end
    Y=fft(y);U=fft(u);
    P=[P U(1:N/2+1).\Y(1:N/2+1)];
end;
% plot results
w=linspace(0,pi,N/2+1);
[m,p]=bode(sys,w);
subplot(2,1,1)
loglog(w,abs(P),'g.',w,abs(m(:)),'r');figure(gcf);axis([1e-2 1e1 1e-2 1])
title('100 ETFEs with periodic input')
ylabel('mag  [gain]')
subplot(2,1,2);
semilogx(w,180/pi*angle(P),'g.',w,p(:),'r');figure(gcf);axis([1e-2 1e1 -200 200])
ylabel('phase  [deg]');
xlabel('w  [rad/s]');
%%




disp('- ETFE with zero pre-initial and non-zero end conditions')
figure(4)
N=2^10;
P=[];
for k=1:100,
    u=randn(N,1);
    %u(1:50)=0*u(1:50);
    y=lsim(sys,u);
    if k==1,clf,plot(1:length(u),u,1:length(y),y),title('I/O signals with zero pre-initial and non-zero end conditions'),legend('input','output'),pause;end
    Y=fft(y);U=fft(u);
    P=[P U(1:N/2+1).\Y(1:N/2+1)];
end;
% plot results
w=linspace(0,pi,N/2+1);
[m,p]=bode(sys,w);
subplot(2,1,1)
loglog(w,abs(P),'g.',w,abs(m(:)),'r');figure(gcf);axis([1e-2 1e1 1e-2 1])
title('100 ETFEs with zero pre-initial and non-zero end conditions')
ylabel('mag  [gain]')
subplot(2,1,2);
semilogx(w,180/pi*angle(P),'g.',w,p(:),'r');figure(gcf);axis([1e-2 1e1 -200 200])
ylabel('phase  [deg]');
xlabel('w  [rad/s]');
%%


disp('- ETFE with non-zero pre-initial and zero end conditions')
figure(5)
N=2^10;
P=[];
for k=1:100,
    u=randn(N+50,1);
    u(N:N+50)=0*u(N:N+50);
    y=lsim(sys,u);
    u=u(51:N+50);
    y=y(51:N+50);
    if k==1,clf,plot(1:length(u),u,1:length(y),y),title('I/O signals with non-zero pre-initial and zero end conditions'),legend('input','output'),pause;end
    Y=fft(y);U=fft(u);
    P=[P U(1:N/2+1).\Y(1:N/2+1)];
end;
% plot results
w=linspace(0,pi,N/2+1);
[m,p]=bode(sys,w);
subplot(2,1,1)
loglog(w,abs(P),'g.',w,abs(m(:)),'r');figure(gcf);axis([1e-2 1e1 1e-2 1])
title('100 ETFEs with non-zero pre-initial and zero end conditions')
ylabel('mag  [gain]')
subplot(2,1,2);
semilogx(w,180/pi*angle(P),'g.',w,p(:),'r');figure(gcf);axis([1e-2 1e1 -200 200])
ylabel('phase  [deg]');
xlabel('w  [rad/s]');
%%

disp('- ETFE with zero pre-initial and zero end conditions')
figure(6)
N=2^10;
P=[];
for k=1:100,
    u=randn(N,1);
    %u(1:50)=0*u(1:50);
    u(N-50:N)=0*u(N-50:N);
    y=lsim(sys,u);
    if k==1,clf,plot(1:length(u),u,1:length(y),y),title('I/O signals with zero pre-initial and zero end conditions'),legend('input','output'),pause;end
    Y=fft(y);U=fft(u);
    P=[P U(1:N/2+1).\Y(1:N/2+1)];
end;
% plot results
w=linspace(0,pi,N/2+1);
[m,p]=bode(sys,w);
subplot(2,1,1)
loglog(w,abs(P),'g.',w,abs(m(:)),'r');figure(gcf);axis([1e-2 1e1 1e-2 1])
title('100 ETFEs with zero pre-initial and zero end conditions')
ylabel('mag  [gain]')
subplot(2,1,2);
semilogx(w,180/pi*angle(P),'g.',w,p(:),'r');figure(gcf);axis([1e-2 1e1 -200 200])
ylabel('phase  [deg]');
xlabel('w  [rad/s]');
%%


disp('- ETFE with burst input')
figure(7)
N=2^10;
P=[];
for k=1:100,
    u=randn(N,1).*hanning(N);
    y=lsim(sys,u);
    if k==1,clf,plot(1:length(u),u,1:length(y),y),title('I/O signals with burst input'),legend('input','output'),pause;end
    Y=fft(y);U=fft(u);
    P=[P U(1:N/2+1).\Y(1:N/2+1)];
end;
% plot results
w=linspace(0,pi,N/2+1);
[m,p]=bode(sys,w);
subplot(2,1,1);
loglog(w,abs(P),'g.',w,abs(m(:)),'r');figure(gcf);axis([1e-2 1e1 1e-2 1])
title('100 ETFEs with burst input')
ylabel('mag  [gain]')
subplot(2,1,2);
semilogx(w,180/pi*angle(P),'g.',w,p(:),'r');figure(gcf);axis([1e-2 1e1 -200 200])
ylabel('phase  [deg]');
xlabel('w  [rad/s]');
%%

disp('Now ETFE with NOISY data (100x)...');

disp('- ETFE with burst input, but now with output noise')
figure(8)
N=2^10;
P=[];
for k=1:100,
    u=randn(N,1).*hanning(N);
    y=lsim(sys,u)+0.01*randn(length(y),1);
    if k==1,clf,plot(1:length(u),u,1:length(y),y),title('I/O signals with burst input and output noise'),legend('input','output'),pause;end
    Y=fft(y);U=fft(u);
    P=[P U(1:N/2+1).\Y(1:N/2+1)];
end;
% plot results
w=linspace(0,pi,N/2+1);
[m,p]=bode(sys,w);
[re,im]=nyquist(sys,w);
subplot(2,1,1);
loglog(w,abs(P),'g.',w,abs(m(:)),'r');figure(gcf);axis([1e-2 1e1 1e-2 1])
title('Bode plot of 100 ETFEs with burst input and output noise')
ylabel('mag  [gain]')
subplot(2,1,2);
semilogx(w,180/pi*angle(P),'g.',w,p(:),'r');figure(gcf);axis([1e-2 1e1 -200 200])
ylabel('phase  [deg]');
xlabel('w  [rad/s]');
%%
clf
plot(real(P),imag(P),'go',re(:),im(:),'r')
title('Nyquist plot of 100 ETFEs with burst input and output noise');
xlabel('Real')
ylabel('Imag')
axis([-0.8 0.4 -0.6 0.2]);
%%


disp('- Same ETFE but now with averaging to obtain a SPA')
figure(9)
N=2^10;
P=[];
for k=1:100,
    u=randn(N,1).*hanning(N);
    y=lsim(sys,u)+0.01*randn(length(y),1);
    if k==1,clf,plot(1:length(u),u,1:length(y),y),title('I/O signals with burst input and output noise'),legend('input','output'),pause;end
    Y=fft(y);U=fft(u);
    
    % Compute Ruu
    Ruu=xcorr(u,u,N/2);
    Ryu=xcorr(y,u,N/2);
    % alternative via fft and ifft
    U=fft(u);
    Y=fft(y);
    Ruu=real(ifft(U.*conj(U)));
    Ruu=Ruu([N/2+1:N 1:N/2+1]);    
    Ryu=real(ifft(Y.*conj(U)));
    Ryu=Ryu([N/2+1:N 1:N/2+1]);
    
    
    % multiply with hanning window of size gamma
    gamma=N/2^2;
    Ruu_weighted = Ruu.*[zeros((N-gamma)/2,1);hanning(gamma+1);zeros((N-gamma)/2,1)];    
    Ryu_weighted = Ryu.*[zeros((N-gamma)/2,1);hanning(gamma+1);zeros((N-gamma)/2,1)];    
    
    % plot Ruu and weighted Ruu
    if k==1,clf,plot(-N/2:N/2,[Ruu/N [zeros((N-gamma)/2,1);hanning(gamma+1);zeros((N-gamma)/2,1)] Ruu_weighted/N]),title(['R_u_u(\tau) without and with Hanning window of width ' num2str(gamma)]),legend('R_u_u(\tau)','Hanning window','R_u_u(\tau)*W(\tau)'),pause;end
    if k==1,clf,plot(-N/2:N/2,[Ryu/N [zeros((N-gamma)/2,1);hanning(gamma+1);zeros((N-gamma)/2,1)] Ryu_weighted/N]),title(['R_y_u(\tau) without and with Hanning window of width ' num2str(gamma)]),legend('R_y_u(\tau)','Hanning window','R_y_u(\tau)*W(\tau)'),pause;end
    
    % make it suitable for fft
    Suu=1/N*fft([Ruu_weighted(N/2+1:N);Ruu_weighted(1:N/2)]);
    Syu=1/N*fft([Ryu_weighted(N/2+1:N);Ryu_weighted(1:N/2)]);
    
    % check if it went O.K.
    if max(abs(imag(Suu)))>1e-13,
        error('FFT of auto specrum must be real valued')
    end    
    Suu=real(Suu);
    
    P=[P Suu(1:N/2+1).\Syu(1:N/2+1)];
end;
% plot results
w=linspace(0,pi,N/2+1);
[m,p]=bode(sys,w);
[re,im]=nyquist(sys,w);
subplot(2,1,1);
loglog(w,abs(P),'g.',w,abs(m(:)),'r');figure(gcf);axis([1e-2 1e1 1e-2 1])
title(['Bode plot of 100 SPAs with Hanning window of width ' num2str(gamma) ' and with burst input and output noise'])
ylabel('mag  [gain]')
subplot(2,1,2);
semilogx(w,180/pi*angle(P),'g.',w,p(:),'r');figure(gcf);axis([1e-2 1e1 -200 200])
ylabel('phase  [deg]');
xlabel('w  [rad/s]');
%%
clf
plot(real(P),imag(P),'go',re(:),im(:),'r')
title(['Nyquist plot of 100 SPAs with Hanning window of width ' num2str(gamma) ' and with burst input and output noise'])
xlabel('Real')
ylabel('Imag')
axis([-0.8 0.4 -0.6 0.2]);
