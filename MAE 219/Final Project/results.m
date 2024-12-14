
x = [-66 -63 -60 -57 -54 -51 -48];
y1 = [1.0 1.0 0.8 0.8 0.5 0.2 0.2];
y2 = [0.6 0.4 0.5 0.6 0.3 0.3 0.4];

fun = @(a,x) 1/2 - erf((x-a(1))/(a(2)*sqrt(2)))/2;

a0 = [-57,1];
a1 = lsqcurvefit(fun,a0,x,y1);
a2 = lsqcurvefit(fun,a0,x,y2);

%%
fun1 = @(x) fun(a1,x);

g = @(x) fun1(x) - 0.25;
x1_25 = fsolve(g,-57);
g = @(x) fun1(x) - 0.50;
x1_50 = fsolve(g,-57);
g = @(x) fun1(x) - 0.75;
x1_75 = fsolve(g,-57);

fun2 = @(x) fun(a2,x);

g = @(x) fun2(x) - 0.25;
x2_25 = fsolve(g,-57);
g = @(x) fun2(x) - 0.50;
x2_50 = fsolve(g,-57);
g = @(x) fun2(x) - 0.75;
x2_75 = fsolve(g,-57);

%%
markersize = 30;
linewidth = 1.5;
fontsize = 20;

x_ = linspace(x(1),x(end),100);

figure(1)
plot(x,y1,'k.',x_,fun1(x_),'b-',MarkerSize=markersize,linewidth=linewidth)
hold on
plot([x1_25 x1_25 x(end)+3],[0 .25 .25],'b:',linewidth=1.5*linewidth)
plot([x1_50 x1_50 x(end)+3],[0 .50 .50],'b--',linewidth=linewidth)
plot([x1_75 x1_75 x(end)+3],[0 .75 .75],'b:',linewidth=1.5*linewidth)
hold off
%legend('Data','Fitted Normal CDF','Location','northwest')
xlabel('Pressure setpoint of comparison stimulus (kPa)')
ylabel('Proportion of "stiffer" responses')
xticks([x x(end)+3])
yticks(0:.1:1)
axis([x(1)-1 x(end)+3 0 1.05])
set(gca,'XDir','reverse','FontSize',fontsize,'LineWidth',linewidth)

%%
figure(2)
plot(x,y2,'k.',x_,fun2(x_),'b-',MarkerSize=markersize,linewidth=linewidth)
hold on
% plot([x2_25 x2_25 x(end)+3],[0 .25 .25],'b:',linewidth=1.5*linewidth)
plot([x2_50 x2_50 x(end)+3],[0 .50 .50],'b--',linewidth=linewidth)
% plot([x2_75 x2_75 x(end)+3],[0 .75 .75],'b:',linewidth=1.5*linewidth)
hold off
%legend('Data','Fitted Normal CDF','Location','northwest')
xlabel('Pressure setpoint of comparison stimulus (kPa)')
ylabel('Proportion of "stiffer" responses')
xticks([x x(end)+3])
yticks(0:.1:1)
axis([x(1)-1 x(end)+3 0 1.05])
set(gca,'XDir','reverse','FontSize',fontsize,'LineWidth',linewidth)