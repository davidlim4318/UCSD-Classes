A = [-1 1 -2; 0 0 -2; 0 0 -2];
B = [3; 2; 1];
C = [1 0 0];

[V,D] = eig(A);

V = [V(:,1)/V(1,1) V(:,2)/V(2,2) V(:,3)/V(3,3)];

A_hat = D;
B_hat = V^-1*B;
C_hat = C*V;

%%
syms t tau
z = int(expm(A_hat*(t-tau))*B_hat*tau,tau,0,t);
y = C_hat*z;

%%
xss = ss(A,B,V^-1,0);

ts = 0:0.001:1;
u = ts;

figure(1)
fplot(z,[0,1])
hold on
plot(ts,lsim(xss,u,ts),'o')
hold off

%%
yss = ss(A,B,C,0);

figure(2)
fplot(y,[0,1])
hold on
plot(ts,lsim(yss,u,ts),'.')
hold off

%% 
figure(3)
fplot(y,[0,1],'LineWidth',2)
set(gca,'FontSize',12,'TickLabelInterpreter','latex')
title('Output y(t)','Interpreter','latex','FontSize',16);
xlabel('t','Interpreter','latex','FontSize',16)
ylabel('y','Interpreter','latex','FontSize',16)