clear

factor = 10;

n = 30;

ms = zeros(5,1);
ds = zeros(5,1);

for i = 1:5
    M = readmatrix("exp2trial" + int2str(i),'Whitespace',[';','[',']']);
    time = M(:,3);
    x1 = M(:,5);
    x2 = M(:,6);
    v1 = diff(x1) / mean(diff(time));
    F = M(:,8);

    figure(2)
    clf
    hold on

    yyaxis left
    plot(time,x1,'b.','MarkerSize',10)
    plot(time,x2,'b--')
    plot(time(1:end-1),factor*v1,'r.','MarkerSize',10)
    ylabel("Displacement (counts)")
    ax = gca;
    ax.YColor = 'b';

    a = time(1:n) \ v1(1:n);
    plot(time(1:n),factor*a*time(1:n),'y-','LineWidth',2)

    idx = find(abs(diff(F)) > 0,1,'last');
    v = mean(v1((idx-n-1):idx));

    yyaxis right
    plot(time,F,'m.','MarkerSize',10)
    ylabel("Force (V)")
    ax = gca;
    ax.YColor = 'm';

    hold off
    title("(a)")
    xlabel("Time (s)")
    legend('Disc 1 Position','Disc 2 Position','Disc 1 Velocity','Disc 1 Acceleration (inital)','Control Effort','Location','best')

    ax = gca;
    ax.TitleHorizontalAlignment = 'left';
    set(ax,'FontSize',12)

    ms(i) = 0.5 / a;
    ds(i) = 0.5 / v;

    pause
end

m = mean(ms);
d = mean(ds);

%%
syms t s
% G = tf(1, [m d 0]);
G = 1 / (m*s^2 + d*s);
u = 0.5 - 0.5*heaviside(t-10);
U = laplace(u);
Y = G*U;
y = ilaplace(Y);

figure(3)
clf
hold on
fplot(y,[0,20],'b-')
for i = 1:5
    M = readmatrix("exp2trial" + int2str(i),'Whitespace',[';','[',']']);
    time = M(:,3);
    x1 = M(:,5);
    plot(time,x1,'r--','MarkerSize',10)
end
hold off
