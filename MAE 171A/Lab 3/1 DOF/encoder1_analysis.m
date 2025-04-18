clear

ks = zeros(5,1);
ms = zeros(5,1);
ds = zeros(5,1);

n = 1;
w = 100;

x1_data = zeros(677,5);

for i = 1:5
    M = readmatrix("encoder1_bottom_trial_" + int2str(i),'Whitespace',[';','[',']']);
    time = M(:,3);
    x1 = M(:,5);
    x2 = M(:,6);
    F = M(:,7);

    x1_data(:,i) = x1;

    [y_pks,t_pks] = findpeaks(x1,time,'NPeaks',n+1);

    idx = find(F < 0.5,1,'first');
    y_ss = mean(x1(idx-w:idx-1));

    omega_d = 2*pi*n/(t_pks(n+1)-t_pks(1));
    beta_omega_n = log((y_pks(1)-y_ss)/(y_pks(n+1)-y_ss))/(t_pks(n+1)-t_pks(1));
    omega_n = sqrt(omega_d^2+(beta_omega_n)^2);
    beta = beta_omega_n/omega_n;

    ks(i) = 0.5/y_ss;
    ms(i) = ks(i)/omega_n^2;
    ds(i) = ks(i)*2*beta/omega_n;
    
    figure(1)
    clf
    hold on
    
    yyaxis left
    plot(time,x1,'b.','MarkerSize',10)
    plot(t_pks,y_pks+10,'bv','MarkerSize',10)
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
    legend('Disc 1','Peaks','Disc 2','Control Effort','Location','best')
    
    ax = gca;
    ax.TitleHorizontalAlignment = 'left';
    set(ax,'FontSize',18)

    pause()
end

k = mean(ks);
m = mean(ms);
d = mean(ds);

%%

x1_mean = mean(x1_data,2);
x1_std = std(x1_data,0,2);

figure(2)
clf

hold on

yyaxis left
e = errorbar(time,x1_mean,x1_std,'b.','MarkerSize',20,'LineWidth',0.25);
e.CapSize = 0;
ylabel("Displacement (counts)")
ax = gca;
ax.YColor = 'k';

yyaxis right
plot(time,F,'mo','MarkerSize',5)
ylabel("Force (V)")
ax = gca;
ax.YColor = 'm';
hold off

xlabel("Time (s)")
title("(a)")
legend('Displacement','Force')  
ax = gca;
ax.TitleHorizontalAlignment = 'left';
set(ax,'FontSize',18)

[y_pks,t_pks] = findpeaks(x1_mean,time,'NPeaks',n+1);

idx = find(F < 0.5,1,'first');
y_ss = mean(x1_mean(idx-w:idx-1));

omega_d = 2*pi*n/(t_pks(n+1)-t_pks(1));
beta_omega_n = log((y_pks(1)-y_ss)/(y_pks(n+1)-y_ss))/(t_pks(n+1)-t_pks(1));
omega_n = sqrt(omega_d^2+(beta_omega_n)^2);
beta = beta_omega_n/omega_n;

k_ = 0.5/y_ss;
m_ = k_/omega_n^2;
d_ = k_*2*beta/omega_n;


%%
x1_mean = mean(x1_data,2);

syms t s
% G = tf(1, [m d 0]);
G = 1 / (m*s^2 + d*s + k);
u = 0.5 - 0.5*heaviside(t-3);
U = laplace(u);
Y = G*U;
y = ilaplace(Y);

figure(3)
clf
hold on
plot(time,x1_mean,'b.','MarkerSize',10)
fplot(y,[0,6],'r-','LineWidth',2)
hold off

ylabel("Displacement (counts)")
xlabel("Time (s)")
title("(b)")
legend('Mean Data','Model')  
ax = gca;
ax.TitleHorizontalAlignment = 'left';
set(ax,'FontSize',18)



%% Revisited
n_pad = 10;
Ts = mean(diff(time));
data = iddata( [zeros(n_pad,1); x1_mean], [zeros(n_pad,1); F], Ts);
data.InputName = 'Voltage';
data.InputUnit = 'volts';
data.OutputName = 'Position';
data.OutputUnit = 'counts';
data.TimeUnit = 'seconds';
data.Tstart = -n_pad*Ts;

% Plot data
figure(4)
clf
plot(data)
set(findall(gcf,'Type','Line'),'LineStyle','none','Marker','.','MarkerSize',10)
%%
% Gss = ssest(data, 1:10);

% Estimate transfer function model
sysTF = tfest(data,3,3);

% Validate transfer function model against data
figure(5)
clf
compare(data,sysTF)

set(findall(gcf,'Type','Line'),'LineWidth',1,'Marker','.','MarkerSize',10)
set(findall(gcf,'Type','Legend'),'Location','Best')