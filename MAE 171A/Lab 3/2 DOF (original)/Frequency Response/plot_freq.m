clear

M = readmatrix("lin_sweep_2.txt",'Whitespace',[';','[',']']);
time = M(:,3);
x2 = M(:,4);
%x2 = M(:,6);
F = M(:,6);

figure(1)
clf
hold on

yyaxis left
plot(time,x2,'b.','MarkerSize',10)
%plot(time,x2,'r.','MarkerSize',10)
ylabel("Displacement (counts)")
ax = gca;
ax.YColor = 'k';

yyaxis right
plot(time,F,'m.','MarkerSize',10)
ylabel("Force (V)")
ax = gca;
ax.YColor = 'm';

hold off
title("(a)")
xlabel("Time (s)")
legend('Disc 1','Disc 2','Control Effort','Location','best')

ax = gca;
ax.TitleHorizontalAlignment = 'left';
set(ax,'FontSize',12)

%%
clear

M = readmatrix("log_sweep_2.txt",'Whitespace',[';','[',']']);
time = M(:,3);
x2 = M(:,4);
%x2 = M(:,6);
F = M(:,6);

figure(2)
clf
hold on

yyaxis left
plot(time,x2,'b.','MarkerSize',10)
%plot(time,x2,'r.','MarkerSize',10)
ylabel("Displacement (counts)")
ax = gca;
ax.YColor = 'k';

yyaxis right
plot(time,F,'m.','MarkerSize',10)
ylabel("Force (V)")
ax = gca;
ax.YColor = 'm';

hold off
title("(a)")
xlabel("Time (s)")
legend('Disc 1','Disc 2','Control Effort','Location','best')

ax = gca;
ax.TitleHorizontalAlignment = 'left';
set(ax,'FontSize',12)

%%
M = readmatrix("lin_sweep.txt",'Whitespace',[';','[',']']);
time = M(:,3);
x2 = M(:,5);
%x2 = M(:,6);
F = M(:,8);

fs_1 = 1/mean(diff(time));
[Txy_1,f_1] = tfestimate(F,x2,[],[],[],fs_1);

M = readmatrix("lin_sweep_2.txt",'Whitespace',[';','[',']']);
time = M(:,3);
x2 = M(:,4);
%x2 = M(:,6);
F = M(:,6);

fs_2 = 1/mean(diff(time));
[Txy_2,f_2] = tfestimate(F,x2,[],[],[],fs_2);


% k2 = 0.003860;
% m1 = 1.711e-05;
% d1 = (1.299e-05 + 1.303e-05)/2;
% m2 = 1.842e-05;
% d2 = 0;

k2 = 0.0020;
m1 = 0.5347e-5;
d1 = 0.1958e-4;
m2 = 0.4650e-4;

G = tf(k2, [m1*m2 (d1*m2 + d2*m1) (d1*d2 + k2*m1 + k2*m2) (d1*k2 + d2*k2) 0]);

[mag, phase] = bode(G,f_1);
a = zeros(length(mag),1);
a(1:end) = mag2db(mag(1,1,1:end));

figure(3)
clf
semilogx(f_1,mag2db(abs(Txy_1)))
hold on
plot(f_2,mag2db(abs(Txy_2)))
plot(f_1,a)

%%
modelfun = @(b,omega) (20*log10(abs ( 1 ./ ( b(5)*(1j*omega).^4 + b(4)*(1j*omega).^3 + b(3)*(1j*omega).^2 + b(2)*(1j*omega) + b(1) ) )));
beta = nlinfit(f_2,Txy_2,modelfun,[1 1 1 1 1]);

G = tf(1, real(beta));

[mag, phase] = bode(G,f_2);
a = zeros(length(mag),1);
a(1:end) = mag2db(mag(1,1,1:end));

plot(f_2,a)

%%
syms m1 m2 d1 k2
eqn1 = m1*m2 == beta(1);
eqn2 = d1*m2 == beta(2);
eqn3 = k2*m1 + k2*m2 == beta(3);
eqn4 = d1*k2 == beta(4);

[m1,m2,d1,k2] = solve([eqn1,eqn2,eqn3,eqn4],[m1,m2,d1,k2]);
m1 = double(m1);
m2 = double(m2);
d1 = double(d1);
k2 = double(k2);