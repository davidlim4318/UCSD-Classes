clear

tiledlayout(5,1)
figure(1)

delete(findall(gcf,'Type','annotation'));

%%
linewidth = 1;
fontsize = 12;

maxP = 85;
x0 = 4;
ax = [0 8 -100 10];
line1 = 'b-';
line2 = 'r--';
line3 = 'r-';
xla = 'd (cm)';
yla = 'P (kPa)';

xapf = @(x,pos) pos(3)*(x-ax(1))/(ax(2)-ax(1))+pos(1);
yapf = @(y,pos) pos(4)*(y-ax(3))/(ax(4)-ax(3))+pos(2);

%%
nexttile(1)
f11 = @(x) -maxP*heaviside(x0-x);
f12 = @(x) -maxP*heaviside(x0-x-2);
fplot(f11,line1,'LineWidth',linewidth)
hold on
fplot(f12,line2,'LineWidth',linewidth)
hold off
title('(a)')
xlabel(xla)
ylabel(yla)
axis(ax)
set(gca,'XDir','reverse','YDir','reverse','FontSize',fontsize,'LineWidth',linewidth,'TitleHorizontalAlignment','left')

%%
nexttile(2)
f21 = @(x) -maxP*heaviside(x0-x);
f22 = @(x) -maxP/2*heaviside(x0-x);
fplot(f21,line1,'LineWidth',linewidth)
hold on
fplot(f22,line2,'LineWidth',linewidth)
hold off
title('(b)')
xlabel(xla)
ylabel(yla)
axis(ax)
set(gca,'XDir','reverse','YDir','reverse','FontSize',fontsize,'LineWidth',linewidth,'TitleHorizontalAlignment','left')

%%
nexttile(3)
f31 = @(x) -maxP*heaviside(x0-x);
f32 = @(x) -maxP*heaviside(x0-x+2);
fplot(f31,line1,'LineWidth',linewidth)
hold on
fplot(f32,line2,'LineWidth',linewidth)
hold off
title('(c)')
xlabel(xla)
ylabel(yla)
axis(ax)
set(gca,'XDir','reverse','YDir','reverse','FontSize',fontsize,'LineWidth',linewidth,'TitleHorizontalAlignment','left')

%%
nexttile(4)
f31 = @(x) -(9*(x0-x)+20)*heaviside(x0-x);
f32 = @(x) -(18*(x0-x)+20)*heaviside(x0-x);
fplot(f31,line1,'LineWidth',linewidth)
hold on
fplot(f32,line2,'LineWidth',linewidth)
hold off
title('(d)')
xlabel(xla)
ylabel(yla)
axis(ax)
set(gca,'XDir','reverse','YDir','reverse','FontSize',fontsize,'LineWidth',linewidth,'TitleHorizontalAlignment','left')

%%
nexttile(5)
f31 = @(x) -maxP*(heaviside(x0-x)-heaviside(x0-x-2));
f32 = @(x) -maxP/2*(heaviside(x0-x)-heaviside(x0-x-2));
fplot(f31,line1,'LineWidth',linewidth)
hold on
fplot(f32,line2,'LineWidth',linewidth)
hold off
title('(e)')
xlabel(xla)
ylabel(yla)
axis(ax)
set(gca,'XDir','reverse','YDir','reverse','FontSize',fontsize,'LineWidth',linewidth,'TitleHorizontalAlignment','left')

%% 
nexttile(1)
hold on
pos = gca().Position;
annotation('arrow',xapf([4 6],pos),yapf([-45 -45],pos),'Color',line3(1),'LineStyle',line3(2),'LineWidth',linewidth);
hold off

nexttile(2)
hold on
pos = gca().Position;
annotation('doublearrow',xapf([6 6],pos),yapf([0 -maxP/2],pos),'Color',line3(1),'LineStyle',line3(2),'LineWidth',linewidth);
hold off

nexttile(3)
hold on
pos = gca().Position;
annotation('doublearrow',xapf([2 4],pos),yapf([-45 -45],pos),'Color',line3(1),'LineStyle',line3(2),'LineWidth',linewidth);
hold off

nexttile(4)
hold on
pos = gca().Position;
annotation('doublearrow',xapf([7.75 7.5],pos),yapf([-35 -5],pos),'Color',line3(1),'LineStyle',line3(2),'LineWidth',linewidth);
hold off

nexttile(5)
hold on
pos = gca().Position;
annotation('doublearrow',xapf([5 5],pos),yapf([0 -maxP/2],pos),'Color',line3(1),'LineStyle',line3(2),'LineWidth',linewidth);
hold off

%% delete(findall(gcf,'Type','annotation'));