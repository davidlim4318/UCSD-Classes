% Notes:
% - system is unstable, output signal drifts
%   - differentiation or high-pass filter on BOTH input and output fixes drift
% - output is encoder signal with a little noise
%   - ETFE result looks good at low frequencies
%   - ETFE result is noisy at high frequencies
% - ETFE requirements NOT satisfied: REPEATED input or with NONZERO end condition
%   - input signal is sinusoidal sweep with non-zero end condition
%   - results in messy data "spray" at high frequencies
% - estimating unstable integrator system with pole = 1
%   - numerical error can make pole > 1, impulse response unbounded
%   - possible fix is differentiating output ONLY
%   - pole cancellation? add pole to model later
% - estimating system with zeros close to 1
%   - numerical error can make zero > 1, response reverses direction
%   - nonminimum phase
% - desired model order affects model stability
% - added DC gain constraint to LS
%   - better fit overall, still some discrepancy at low frequencies
%   - data might not have enough low frequency information
% - SPA does yield good result with large Hanning window
% - tried LS constraint on full system dynamics
%   - seemed to fit integrator gain well
%   - comparing ysim and y revealed DC gain discrepancy, still an issue

% clear
% M = readmatrix("lin_sweep_2.txt",'Whitespace',[';','[',']']);
% t = M(:,3);
% y = M(:,4); % x1
% u = M(:,6);

clear
M = readmatrix("lin_sweep.txt",'Whitespace',[';','[',']']);
t = M(:,3);
y = M(:,5); % x1
u = M(:,8);

N = length(t);
Ts = mean(diff(t));

uf = u;
yf = y;

%% Prefilter
% [num1,den1] = butter(1,.1,"high");
% num = num1;
% den = den1;

% [num2,den2] = butter(5,.2);
% num = conv(num1,num2);
% den = conv(den1,den2);

% figure(1)
% [h,w] = freqz(num1,den1);
% loglog(w,abs(h))
% xlabel('Frequency (rad/sample)')
% ylabel('Magnitude')

% uf = filter(num,den,u);
% yf = filter(num,den,y);

num = [1 -1];
% den = [1 0];
den = 1;
uf = u;
yf = filter(num,den,y);

m = 2000;
figure(2)
plot([u(1:m)/max(abs(u(1:m))),y(1:m)/max(abs(y(1:m)))])
figure(3)
plot([uf(1:m)/max(abs(uf(1:m))),yf(1:m)/max(abs(yf(1:m)))])

%%
[G, w] = tfestimate(uf,yf,rectwin(N),0,N);

Y = fft(yf);
U = fft(uf);
P = U(1:N/2+1).\Y(1:N/2+1);

%%
figure(4)
clf
loglog(w,abs(P),'o',w,abs(G),'.','LineWidth',1);

xlabel("Frequency (rad/s)")
ylabel("Magnitude")
title('Amplitude Bode plot')
ax = gca;
ax.TitleHorizontalAlignment = 'left';
set(ax,'FontSize',18)
legend('G_{etfe}','tfestimate')

%% 
save("Getfe.mat","P")

% %%
% gamma = 100;
% Ruu = xcorr(uf,uf,N/2);
% Ryu = xcorr(yf,uf,N/2);
% W = [zeros((N-gamma)/2,1);hanning(gamma+1);zeros((N-gamma)/2,1)];
% Ruu_weighted = Ruu.*W;    
% Ryu_weighted = Ryu.*W;
% 
% figure(5)
% plot(0:gamma,Ryu(N/2+1:N/2+gamma+1)/N,0:gamma,W(N/2+1:N/2+gamma+1),0:gamma,Ryu_weighted(N/2+1:N/2+gamma+1)/N,'LineWidth',1);
% 
% %%
% Suu = 1/N*fft([Ruu_weighted(N/2+1:N);Ruu_weighted(1:N/2)]);
% Syu = 1/N*fft([Ryu_weighted(N/2+1:N);Ryu_weighted(1:N/2)]);
% Pspa = Suu(1:N/2+1).\Syu(1:N/2+1);
% 
% %%
% w = linspace(0,pi,N/2+1);
% 
% figure(6)
% clf
% loglog(w,abs(P),'o',w,abs(Pspa),'.','LineWidth',1);
% 
% xlabel("Frequency (rad/s)")
% ylabel("Magnitude")
% title('Amplitude Bode plot')
% ax = gca;
% ax.TitleHorizontalAlignment = 'left';
% set(ax,'FontSize',18)
% legend('G_{etfe}','G_{spa}')