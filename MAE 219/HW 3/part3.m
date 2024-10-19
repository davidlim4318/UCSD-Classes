clear
count = [-2126 -1390 -644 206 864 1640 2490 3150 3847]';
offset = @(x) x - 864 + 520;
count = offset(count);

angle = (-40:10:40)';

f = fit(count,angle,'poly1');
b = f.p1;
a = f.p2;

g = @(x) b*x + a;

angle_fit = g(count);

%%
figure(1)
clf
plot(count,angle,'LineStyle','none','Marker','.','MarkerSize',15);
hold on
plot(count,angle_fit,'LineStyle','-','LineWidth',2)
hold off
axis padded

title('Handle Angle vs. MR Position')
xlabel('position (counts)')
ylabel('angle (deg)')
legend({'data',sprintf('linear fit: y = (%.5f)*x + (%.3f)',b,a)},'Location','best')
set(gca,'FontSize',16)