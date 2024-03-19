clear

M = readmatrix("lin_sweep.txt",'Whitespace',[';','[',']']);
time = M(:,3);
x2 = M(:,5);
%x2 = M(:,6);
F = M(:,8);

figure(1)
clf
hold on

yyaxis left
plot(time,x2,'b.','MarkerSize',10)
%plot(time,x2,'r.','MarkerSize',10)
ylabel("Displacement (counts)")
ax = gca;
ax.YColor = 'k';

plot(time,smoothdata(x2),'r-','LineWidth',2)

% yyaxis right
% plot(time,F,'m.','MarkerSize',10)
% ylabel("Force (V)")
% ax = gca;
% ax.YColor = 'm';

hold off
title("(a)")
xlabel("Time (s)")
legend('Disc 1','Disc 2','Control Effort','Location','best')

ax = gca;
ax.TitleHorizontalAlignment = 'left';
set(ax,'FontSize',12)

%%
clear

M = readmatrix("lin_sweep_2.txt",'Whitespace',[';','[',']']);
time = M(:,3);
x2 = M(:,4);
%x2 = M(:,6);
F = M(:,6);

figure(2)
clf
hold on

yyaxis left
plot(time,x2,'b.','MarkerSize',10)
%plot(time,x2,'r.','MarkerSize',10)
ylabel("Displacement (counts)")
ax = gca;
ax.YColor = 'k';

plot(time,smoothdata(x2),'r-','LineWidth',2)

% yyaxis right
% plot(time,F,'m.','MarkerSize',10)
% ylabel("Force (V)")
% ax = gca;
% ax.YColor = 'm';

hold off
title("(a)")
xlabel("Time (s)")
legend('Disc 1','Disc 2','Control Effort','Location','best')

ax = gca;
ax.TitleHorizontalAlignment = 'left';
set(ax,'FontSize',12)

%%
M = readmatrix("lin_sweep.txt",'Whitespace',[';','[',']']);
time = M(:,3);
x2 = M(:,5);
%x2 = M(:,6);
F = M(:,8);

fs_1 = 1/mean(diff(time));
[Txy_1,f_1] = tfestimate(F,x2,[],[],[],fs_1);

%Txy_1 = smoothdata(Txy_1,'lowess',20);

M = readmatrix("lin_sweep_2.txt",'Whitespace',[';','[',']']);
time = M(:,3);
x2 = M(:,4);
%x2 = M(:,6);
F = M(:,6);

fs_2 = 1/mean(diff(time));
[Txy_2,f_2] = tfestimate(F,x2,[],[],[],fs_2);

%Txy_2 = smoothdata(Txy_2,'lowess',20);

k2 = 2.710e-3;
m1 = 1.964e-6;
d1 = 1.630e-5;
m2 = 6.076e-6;
d2 = 1.657e-6;

G0 = tf(k2, [m1*m2 (d1*m2 + d2*m1) (d1*d2 + k2*m1 + k2*m2) (d1*k2 + d2*k2) 0]);
G = 10000*tf( [1 2*0.02*3 3^2] , conv( conv([1 0],[1 2*0.03*4 4^2]) , [1 0.1] ) );
% G = 350000000*tf( [1 2*0.02*3 3^2] , conv ( conv( conv([1 0],[1 2*0.03*4 4^2]) , [1 2*0.05*45 45^2] ) , [1 2]) );

[mag, ~] = bode(G,f_1);
a = zeros(length(mag),1);
a(1:end) = mag(1,1,1:end);

[mag, ~] = bode(G0,f_1);
b = zeros(length(mag),1);
b(1:end) = mag(1,1,1:end);

figure(3)
clf
semilogx(f_1,mag2db(abs(Txy_1)))
hold on
plot(f_2,mag2db(abs(Txy_2)))
plot(f_1,mag2db(a),'LineWidth',5)
plot(f_1,mag2db(b),'LineWidth',5)

% syms s b1 b2 b3 b4 b5 b6 omega
% G_sym = b6 * (s + b5) * (s^2 + 2*b4*s + b3^2) / ( s^2 * (s^2 + 2*b2*s + b1^2));

%{
%%
options = statset('MaxIter',10000);

%modelfun = @(b,omega) ( abs( b(6) .* (1j.*omega + b(5)) .* ((1j.*omega).^2 + 2.*b(4).*1j.*omega + b(3).^2) ./ ( (1j.*omega).^2 .* ((1j.*omega).^2 + 2.*b(2).*1j.*omega + b(1).^2 ) ) ) );
%beta = nlinfit(f_2(1:1000),Txy_2(1:1000),modelfun,[4 0.03*4 3 0.05*3 15 200], options);

modelfun = @(b,omega) ( abs ( (b(1).*(1j.*omega).^3 - b(2).*omega.^2 + b(3).*1j.*omega + b(4)) ./ (omega.^4 + b(5).*(1j.*omega).^3 - b(6).*omega.^2) ) );
beta = nlinfit(f_2(1:1000),Txy_2(1:1000),modelfun,[200 3060 2700 27000 0.24 16], options);

G = beta(6)*tf( conv([1 beta(5)],[1 2*beta(4) beta(3)^2]) , conv([1 0 0],[1 2*beta(2) beta(1)^2]) );

[mag, ~] = bode(G,f_2);
a = zeros(length(mag),1);
a(1:end) = mag(1,1,1:end);

plot(f_2,mag2db(a))
%}
