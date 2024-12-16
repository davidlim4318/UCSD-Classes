%% 
clear
% G0 = 4000*tf( [1 2*0.02*3 3^2] , conv( conv([1 0],[1 2*0.03*4 4^2]) , [1 0.1] ) );
% G0 = c2d(G0,0.01,'zoh');
G0 = 4000*tf( [1 2] , conv( conv([1 0],[1 2*0.03*4 4^2]) , [1 0.5] ) );
G0 = c2d(G0,0.05,'zoh');
G0.Ts = 1;
bode(G0)

% N = 2048;
% u = randn(N,1);
% u = u.*hanning(N);
% y = lsim(G0,u);

%%
load("input.mat")
N = length(u);

lambda = 3;
[numh,denh] = butter(2,0.1,"low");
H0 = tf(numh,denh,1);

e = sqrt(lambda)*randn(N,1);
v = lsim(H0,e);
plot(v)
shg

%%
y = lsim(G0,u) + v;

uf = u;
yf = y;

%% Prefilter
[num1,den1] = butter(1,.01,"high");
num = num1;
den = den1;
% [num2,den2] = butter(5,.2);
% num = conv(num1,num2);
% den = conv(den1,den2);

% figure(1)
% [h,w] = freqz(num1,den1);
% loglog(w,abs(h))
% xlabel('Frequency (rad/sample)')
% ylabel('Magnitude')

uf = filter(num,den,u);
yf = filter(num,den,y);

num = [1 -1];
den = [1 0];
uf = u;
yf = filter(num,den,y);

figure(2)
plot([u/max(abs(u)),y/max(abs(y))])
figure(3)
plot([uf/max(abs(uf)),yf/max(abs(yf))])

%%
[G, w] = tfestimate(uf,yf,rectwin(N),0,N);
% Gidfrd = etfe([yf,uf]);

Y = fft(yf);
U = fft(uf);
P = U(1:N/2+1).\Y(1:N/2+1);

%%
figure(4)
clf
loglog(w,abs(P),'-',w,abs(G),'-','LineWidth',1);

[M,~] = bode(G0,w);
hold on
plot(w,M(:),'LineWidth',2)
hold off

xlabel("Frequency (rad/s)")
ylabel("Magnitude")
title('Amplitude Bode plot of SPA')
ax = gca;
ax.TitleHorizontalAlignment = 'left';
set(ax,'FontSize',18)
legend('G_{etfe}','tfestimate')

%%
save("Getfe.mat","P","G0")