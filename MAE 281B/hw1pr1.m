%% MAE 281B, Homework 1, Problem 1 (iii)

% By David Lim, A16398479
% 04/07/25
clear

t = 0:0.01:10;
N = length(t);

k = [2 2];  % gain
x0 = [1 1];  % initial condition

[~,x_sat] = ode45(@(T,x) sys_sat(T,x,k),t,x0);  % simulate with saturation
[~,x_nosat] = ode45(@(T,x) sys_nosat(T,x,k),t,x0);  % simulate without

figure(1)  % plot results
tiledlayout(1,2)
nexttile(1)
plot(t,x_sat(:,1),'LineWidth',2)
hold on
plot(t,x_nosat(:,1),':','LineWidth',2)
hold off
title('Trajectory $$x_1(t)$$','Interpreter','latex')
xlabel('$$t$$','Interpreter','latex')
ylabel('$$x_1(t)$$','Interpreter','latex')
legend('With saturation','Without saturation','Interpreter','latex')
set(gca,'FontSize',16,'TickLabelInterpreter','latex')
axis square
nexttile(2)
plot(t,x_sat(:,2),'LineWidth',2)
hold on
plot(t,x_nosat(:,2),':','LineWidth',2)
hold off
title('Trajectory $$x_2(t)$$','Interpreter','latex')
xlabel('$$t$$','Interpreter','latex')
ylabel('$$x_2(t)$$','Interpreter','latex')
legend('With saturation','Without saturation','Interpreter','latex')
set(gca,'FontSize',16,'TickLabelInterpreter','latex')
axis square

function dxdt = sys_sat(~,x,k)  % system with saturation
dxdt = zeros(2,1);
dxdt(1) = x(2);
u = - k(1)*x(1) - k(2)*x(2);
dxdt(2) = sign(u)*min(1,abs(u));
end

function dxdt = sys_nosat(~,x,k)  % system without saturation
dxdt = zeros(2,1);
dxdt(1) = x(2);
u = - k(1)*x(1) - k(2)*x(2);
dxdt(2) = u;
end