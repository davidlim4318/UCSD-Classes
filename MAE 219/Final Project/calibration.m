clear

x = [0 100 200 300 400 500 600 700 800 900 1000 1100 1200 1300 1400 1500 1600 1700]';
y = [-86 -83 -72 -62 -51 -39 -27 -15 -5 0 8 23 31 44 54 65 77 90]';
b = ones(length(x),1);

theta = [x b]\y;

fun = @(x) theta(1)*x + theta(2);

%%
markersize = 30;
linewidth = 1.5;
fontsize = 20;

x_ = linspace(x(1),x(end),100);

figure(1)
plot(x,y,'k.',x_,fun(x_),'b-',MarkerSize=markersize,linewidth=linewidth)
%legend('Data','Fitted Normal CDF','Location','northwest')
xlabel('Voltage command (mV)')
ylabel('Pressure output (kPa)')
% xticks([x x(end)])
% yticks(0:.1:1)
% axis([x(1)-1 x(end)+3 0 1.05])
axis padded
set(gca,'FontSize',fontsize,'LineWidth',linewidth)