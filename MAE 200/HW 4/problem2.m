%% Homework 4, Problem 2

% David Lim
% A16398479
% 02/21/25
clear

%% Part b.
A = [0 1 0 0; -11 -0.2 10 0.1; 0 0 0 1; 10 0.1 -10 -0.1];
[V,D] = eig(A);
D

p1 = (V(:,1)+V(:,2))/2;
p2 = (V(:,1)-V(:,2))/2j;
p3 = (V(:,3)+V(:,4))/2;
p4 = (V(:,3)-V(:,4))/2j;

a1 = real(D(1,1));
b1 = imag(D(1,1));
a2 = real(D(3,3));
b2 = imag(D(3,3));

Vnew = [p1 p2 p3 p4];
Dnew = [a1 b1 0 0; -b1 a1 0 0; 0 0 a2 b2; 0 0 -b2 a2];
% Alternatively, use:
% [Vnew1,Dnew1] = cdf2rdf(V,D)

X = Vnew
Anew = Dnew

%% Part c.
syms t

q01 = [1 1 0 0]';
q02 = [0 0 1 1]';

q1 = expm(Dnew*t)*q01;
q1 = vpa(simplify(q1,'Steps',9))

q2 = expm(Dnew*t)*q02;
q2 = vpa(simplify(q2,'Steps',9))

x1 = X*q1;
x2 = X*q2;

figure(1)
fplot(q1,[0,10])
legend('q_1(t)','q_2(t)','q_3(t)','q_4(4)')
xlabel('t')
ylabel('q(t)')
title("q(t) for inital condition q_0 = [1 1 0 0]'")
set(gca,'FontSize',12)

figure(2)
fplot(q2,[0,10])
legend('q_1(t)','q_2(t)','q_3(t)','q_4(4)')
xlabel('t')
ylabel('q(t)')
title("q(t) for inital condition q_0 = [0 0 1 1]'")
set(gca,'FontSize',12)

figure(3)
fplot(x1(1),x1(3),[0,10])
axis equal
xlabel('y')
ylabel('z')
title("Trajectory in y-z plane for inital condition q_0 = [1 1 0 0]'")
set(gca,'FontSize',12)

figure(4)
fplot(x2(1),x2(3),[0,10])
axis equal
xlabel('y')
ylabel('z')
title("Trajectory in y-z plane for inital condition q_0 = [0 0 1 1]'")
set(gca,'FontSize',12)

%% Check answer with ODE45
tspan = 0:0.01:10;

m1 = 1;
m2 = 1;
k1 = 1;
k2 = 10;
c1 = 0.1;
c2 = 0.1;

[~,x3] = ode45(@(t,x) sys(t,x,m1,m2,k1,k2,c1,c2), tspan, X*q01);
[~,x4] = ode45(@(t,x) sys(t,x,m1,m2,k1,k2,c1,c2), tspan, X*q02);

figure(5)
clf
hold on
fplot(x1(1),[0,tspan(end)])
fplot(x1(3),[0,tspan(end)])
plot(tspan,x3(:,1),'--')
plot(tspan,x3(:,3),'--')
hold off
legend('1','2','3','4')

figure(6)
clf
hold on
fplot(x2(1),[0,tspan(end)])
fplot(x2(3),[0,tspan(end)])
plot(tspan,x4(:,1),'--')
plot(tspan,x4(:,3),'--')
hold off
legend('1','2','3','4')

%% Animate
tspan = 0:0.1:100;

m1 = 1;
m2 = 1;
k1 = 1;
k2 = 10;
c1 = 0.1;
c2 = 0.1;

[~,x] = ode45(@(t,x) sys(t,x,m1,m2,k1,k2,c1,c2), tspan, [0 0 0.5 -0.5]);

figure(7)
clf
hold on
plot(tspan,x(:,1),'-')
plot(tspan,x(:,3),'-')
hold off
legend('1','2','3','4')

figure(8)
clf
h = animatedline('Linewidth',2,'Marker','square','MarkerSize',20);
v1 = animatedline('Linewidth',2,'Color','g');
v2 = animatedline('Linewidth',2,'Color','g');
vscale = 0.5;
axis([-1 2 -0.5 0.5])
shg

for k = 1:length(tspan)
    clearpoints(h)
    clearpoints(v1)
    clearpoints(v2)
    addpoints(h,[0 x(k,1)+0.5 x(k,3)+1],[0 0 0]);
    addpoints(v1,[x(k,1)+0.5 x(k,1)+0.5+vscale*x(k,2)],[0 0]);
    addpoints(v2,[x(k,3)+1 x(k,3)+1+vscale*x(k,4)],[0 0]);
    drawnow
    if k == 1
        pause
    end
end