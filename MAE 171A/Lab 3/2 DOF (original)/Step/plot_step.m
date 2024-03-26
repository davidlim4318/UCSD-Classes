clear

M = readmatrix("trial1.txt",'Whitespace',[';','[',']']);
time = M(:,3);
x1 = M(:,5);
x2 = M(:,6);
F = M(:,8);

figure(1)
clf
hold on

yyaxis left
plot(time,x1,'b.','MarkerSize',10)
plot(time,x2,'r.','MarkerSize',10)
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
k2 = 0.003860;
m1 = 1.711e-05;
d1 = (1.299e-05 + 1.303e-05)/2;
m2 = 1.842e-05;
d2 = 0;

syms t s
G = k2/(m1*m2*s^4 + (d1*m2 + d2*m1)*s^3 + (d1*d2 + k2*m1 + k2*m2)*s^2 + (d1*k2 + d2*k2)*s);
u = 0.5 - 0.5*heaviside(t-10);
U = laplace(u);
Y = G*U;
y = ilaplace(Y);

figure(2)
clf
hold on

fplot(y,[0,20],'b-')

M = readmatrix("trial2.txt",'Whitespace',[';','[',']']);
time = M(:,3);
x2 = M(:,6);
plot(time,x1,'r--','MarkerSize',10)

hold off
