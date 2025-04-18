clear

figure(1)
clf
hold on

for i = 1:1

    M = readmatrix("2DOF_" + int2str(i),'Whitespace',[';','[',']']);
    time = M(:,3);
    x1 = M(:,5);
    x2 = M(:,6);
    F = M(:,7);
    
    yyaxis left
    %%plot(time,x1,'b.','MarkerSize',10)
    plot(time,x2,'b.','MarkerSize',10)
    ylabel("Displacement (counts)")
    ax = gca;
    ax.YColor = 'k';
   
end

%%
k2 = 2.710e-3;
m1 = 5.783e-6;
d1 = 5.885e-5;
m2 = 7.441e-6;
d2 = 7.772e-6;

syms t s
G = k2/(m1*m2*s^4 + (d1*m2 + d2*m1)*s^3 + (d1*d2 + k2*m1 + k2*m2)*s^2 + (d1*k2 + d2*k2)*s);
% G = 10000 * (s^2 + 2*0.02*3 + 3^2) / (s * (s^2 + 2*0.03*4 + 4^2) * (s + 0.1));
% G = 350000000 * (s^2 + 2*0.02*3*s + 3^2) / ( s * (s^2 + 2*0.03*4*s + 4^2) * (s^2 + 2*0.05*45 + 45^2) * (s + 2) );

u = 0.5 - 0.5*heaviside(t-3);
U = laplace(u);
Y = G*U;
y = ilaplace(Y);

yyaxis left
hold on

fplot(y,[0,6],'r--','LineWidth',2)

yyaxis right
plot(time,F,'mo','MarkerSize',5)
ylabel("Force (V)")
ax = gca;
ax.YColor = 'm';

hold off

title("(a)")
xlabel("Time (s)")
legend('Data','Model','Control Effort','Location','best')

ax = gca;
ax.TitleHorizontalAlignment = 'left';
set(ax,'FontSize',18)



%% Revisited
n_pad = 10;
Ts = mean(diff(time));
data = iddata( [zeros(n_pad,1); x1], [zeros(n_pad,1); F], Ts);
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
Gss = ssest(data, 1:10);
%%
% Estimate transfer function model
sysTF = tfest(data,4,3);

% Validate transfer function model against data
figure(5)
clf
compare(data,sysTF)

set(findall(gcf,'Type','Line'),'LineWidth',1,'Marker','.','MarkerSize',10)
set(findall(gcf,'Type','Legend'),'Location','Best')

figure(6)
clf
bode(sysTF)
