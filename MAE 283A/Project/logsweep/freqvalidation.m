clear

M = readmatrix("log_sweep.txt",'Whitespace',[';','[',']']);
t = M(:,3);
y = M(:,5); % x1
u = M(:,8);

N = length(t);
Ts = mean(diff(t));

load("G1a3b4d0i15.mat")

%%
w = linspace(0,pi,N/2+1);
[mag,~] = bode(G1,w);

load("Getfe.mat")

figure(1)
clf
loglog(w,abs(P),'o',w,mag(:),'.','LineWidth',1);
xlabel("Frequency (rad/s)")
ylabel("Magnitude")
title('Amplitude Bode plot')
ax = gca;
ax.TitleHorizontalAlignment = 'left';
set(ax,'FontSize',18)
legend('data','model')

%% 
ysim = lsim(G1,u);

m = N;
figure(2)
plot([y(1:m),ysim(1:m)])
legend('data','sim')

%%
M1 = readmatrix("2DOF_1.txt",'Whitespace',[';','[',']']);
t1 = M1(:,3);
y1 = M1(:,5); % x1
u1 = M1(:,7);

ysim1 = lsim(G1,u1);

figure(3)
plot([y1,ysim1])
legend('data','sim')
