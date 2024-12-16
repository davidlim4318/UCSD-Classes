clear

[num,den] = butter(1,.01,"high");

M1 = readmatrix("lin_sweep.txt",'Whitespace',[';','[',']']);
y1 = filter(num,den,M1(:,5));
u1 = filter(num,den,M1(:,8));
N1 = length(u1);

M2 = readmatrix("log_sweep.txt",'Whitespace',[';','[',']']);
y2 = filter(num,den,M2(:,5));
u2 = filter(num,den,M2(:,8));
N2 = length(u2);

M3 = readmatrix("lin_sweep_2.txt",'Whitespace',[';','[',']']);
y3 = filter(num,den,M3(:,4));
u3 = filter(num,den,M3(:,6));
N3 = length(u3);

M4 = readmatrix("log_sweep_2.txt",'Whitespace',[';','[',']']);
y4 = filter(num,den,M4(:,4));
u4 = filter(num,den,M4(:,6));
N4 = length(u4);

%%
[G1, w1] = tfestimate(u1,y1,rectwin(N1),0,N1);
[G2, w2] = tfestimate(u2,y2,rectwin(N2),0,N2);
[G3, w3] = tfestimate(u3,y3,rectwin(N3),0,N3);
[G4, w4] = tfestimate(u4,y4,rectwin(N4),0,N4);

%%
figure(4)
clf
loglog(w1,abs(G1),w2,abs(G2),w3,abs(G3),w4,abs(G4),'LineWidth',1);

xlabel("Frequency (rad/s)")
ylabel("Magnitude")
title('Amplitude Bode plot')
ax = gca;
ax.TitleHorizontalAlignment = 'left';
set(ax,'FontSize',18)
legend('lin_sweep','log_sweep','lin_sweep_2','log_sweep_2','Location','best')