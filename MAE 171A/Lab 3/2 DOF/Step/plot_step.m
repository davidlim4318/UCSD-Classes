clear

figure(1)
clf
hold on

for i = 1:3

    M = readmatrix("2DOF_" + int2str(i),'Whitespace',[';','[',']']);
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
   
end

%%
k2 = 2.710e-3;
m1 = 1.964e-6;
d1 = 1.630e-5;
m2 = 6.076e-6;
d2 = 1.657e-6;

syms t s
%G = k2/(m1*m2*s^4 + (d1*m2 + d2*m1)*s^3 + (d1*d2 + k2*m1 + k2*m2)*s^2 + (d1*k2 + d2*k2)*s);
%G = (200*s^3 + 3060*s^2 + 2700*s + 27000) / (s^4 + 0.24*s^3 +16*s^2);

u = 0.5 - 0.5*heaviside(t-3);
U = laplace(u);
Y = G*U;
y = ilaplace(Y);

yyaxis left
fplot(y,[0,6],'b-')

hold off

title("(a)")
xlabel("Time (s)")
legend('Disc 1','Disc 2','Control Effort','Location','best')

ax = gca;
ax.TitleHorizontalAlignment = 'left';
set(ax,'FontSize',12)
