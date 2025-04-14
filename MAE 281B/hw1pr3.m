%% MAE 281B, Homework 1, Problem 3

% By David Lim, A16398479
% 04/10/25
clear

a = 10;  % given parameters
c = 10;
b = 0;
theta_des = pi/4;
k1 = 2.5;
k2 = 1;

A = [0 1; -c*k1-a*cos(theta_des) -b-c*k2];  % linearization

syms P [2 2]
P(2,1) = P(1,2);
Q = eye(2);
eqn = P*A + A'*P == -Q;
[P1_1,P1_2,P2_2] = solve(eqn,[P1_1,P1_2,P2_2]);  % recompute P

P = [double(subs(P1_1)) double(subs(P1_2));
     0 double(subs(P2_2))];
P(2,1) = P(1,2)

% Compute parameters
gamma = 1/2*min(eig(Q))/norm(P)
r = 2*gamma/a
C = min(eig(P))*r^2  % region of attraction estimate parameter

%%
x0 = [pi pi; pi/2 pi; 0 pi; -pi/2 pi; -pi pi; -pi -pi; -pi/2 -pi; 0 -pi; pi/2 -pi; pi -pi];  % initial conditions
dim = size(x0);
t = 0:0.01:2;

figure(1)
clf
hold on

syms x1 x2
fimplicit([x1 x2]*P*[x1;x2] == C,'b-','LineWidth',2)  % plot region of attraction estimate

for i = 1:dim(1)  % compute and plot solution for each initial condition
    [~,xsol] = ode45(@(t,x)sys(t,x,a,b,c,theta_des,k1,k2),t,x0(i,:));
    plot(xsol(:,1),xsol(:,2),'r-','LineWidth',1)
end

[X1,X2] = meshgrid(linspace(-pi,pi,12),linspace(-2*pi,2*pi,24));
X1dot = zeros(size(X1));
X2dot = zeros(size(X2));
for i = 1:numel(X1)
    dxdt = sys(0,[X1(i) X2(i)],a,b,c,theta_des,k1,k2);  % compute vector at specific points
    X1dot(i) = dxdt(1);
    X2dot(i) = dxdt(2);
end
quiver(X1,X2,X1dot,X2dot,'r')  % plot vector field

hold off
title('Trajectories of Forced Pendulum with Control','Interpreter','latex')
xlabel('$$x_1$$','Interpreter','latex')
ylabel('$$x_2$$','Interpreter','latex')
legend('Region of attraction','Trajectories','Interpreter','latex')
set(gca,'FontSize',16,'TickLabelInterpreter','latex')

function dxdt = sys(~,x,a,b,c,theta_des,k1,k2)  % system
dxdt = zeros(2,1);
dxdt(1) = x(2);
dxdt(2) = -a*(sin(x(1)+theta_des) - sin(theta_des)) - c*k1*x(1) - (b+c*k2)*x(2);
end
