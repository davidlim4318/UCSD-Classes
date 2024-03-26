clear

figure(1)
clf
hold on

M = readmatrix("kp_018",'Whitespace',[';','[',']']);
time = M(:,3);
x1 = M(:,5);
x2 = M(:,6);
F = M(:,7);

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
