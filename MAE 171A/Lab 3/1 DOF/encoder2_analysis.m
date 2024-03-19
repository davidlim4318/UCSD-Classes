clear

k = 0.002710;
ms = zeros(5,1);
ds = zeros(5,1);

n = 10;

x2_data = zeros(677,5);

for i = 1:5
    M = readmatrix("encoder2_top_trial_" + int2str(i),'Whitespace',[';','[',']']);
    time = M(:,3);
    x1 = -M(:,5);
    x2 = -M(:,6);
    F = M(:,7);
    
    idx = find(x2 == max(x2),1,'first');
    [y_pks,t_pks] = findpeaks(x2(idx:end),time(idx:end),'NPeaks',n+1);

    y_ss = mean(x2(idx:find(time == t_pks(n+1))));

    x2_data(1:(677-idx+1),i) = x2(idx:end) - y_ss;

    omega_d = 2*pi*n/(t_pks(n+1)-t_pks(1));
    beta_omega_n = log((y_pks(1)-y_ss)/(y_pks(n+1)-y_ss))/(t_pks(n+1)-t_pks(1));
    omega_n = sqrt(omega_d^2+(beta_omega_n)^2);
    beta = beta_omega_n/omega_n;

    ms(i) = k/omega_n^2;
    ds(i) = k*2*beta/omega_n;
    
    figure(1)
    clf
    hold on
    
    yyaxis left
    plot(time,x1,'b.','MarkerSize',10)
    plot(t_pks,y_pks+15,'bv','MarkerSize',10)
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
    set(ax,'FontSize',12)

    pause()
end

m = mean(ms);
d = mean(ds);

%%

x2_mean = mean(x2_data,2);
x2_std = std(x2_data,0,2);

figure(2)
clf

hold on

yyaxis left
e = errorbar(time,x2_mean,x2_std,'b.','MarkerSize',20,'LineWidth',0.25);
e.CapSize = 0;
ylabel("Displacement (counts)")
ax = gca;
ax.YColor = 'k';

xlabel("Time (s)")
title("(a)")
legend('Displacement')  
ax = gca;
ax.TitleHorizontalAlignment = 'left';
set(ax,'FontSize',12)

idx = find(x2_mean == max(x2_mean),1,'first');
[y_pks,t_pks] = findpeaks(x2_mean(idx:end),time(idx:end),'NPeaks',n+1);

y_ss = mean(x2_mean(idx:find(time == t_pks(n+1))));

x2_data(1:(677-idx+1),i) = x2_mean(idx:end) - y_ss;

omega_d = 2*pi*n/(t_pks(n+1)-t_pks(1));
beta_omega_n = log((y_pks(1)-y_ss)/(y_pks(n+1)-y_ss))/(t_pks(n+1)-t_pks(1));
omega_n = sqrt(omega_d^2+(beta_omega_n)^2);
beta = beta_omega_n/omega_n;

m_ = k/omega_n^2;
d_ = k*2*beta/omega_n;
