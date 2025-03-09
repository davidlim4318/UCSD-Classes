%% Problem 1(1)
A1 = sym([0 1; -1 -1]);
eig(A1)
A2 = sym([0 1; 2 -1]);
eig(A2)

%%
figure(1)
clf
hold on

xeq = [0 0; sqrt(6) 0; -sqrt(6) 0];
plot(xeq(:,1),xeq(:,2),'ko')

N = 12;
R = 5;
w = linspace(0,2*pi,N+1);
x0 = [R*cos(w); R*sin(w)];
tspan = 0:0.1:5;

for i = 1:N
    [t,x] = ode45(@sys1a,tspan,x0(:,i));
    plot(x(:,1),x(:,2),'b.-')
end

[X1,X2] = meshgrid(-R-1:R+1,-R-1:R+1);
X1dot = zeros(size(X1));
X2dot = zeros(size(X2));
for i = 1:numel(X1)
    dxdt = sys1a(0,[X1(i) X2(i)]);
    X1dot(i) = dxdt(1);
    X2dot(i) = dxdt(2);
end
quiver(X1,X2,X1dot,X2dot,'r')

hold off
axis([-R-2 R+2 -R-2 R+2])
axis square

%% Problem 1(2)
g = @(x1) 0.1 - 2*x1 - 0.3*x1^2;

A1 = [-1 1; 0.1 -2];
eig(A1)

g(-5+sqrt(6))
A2 = [-1 1; g(-5+sqrt(6)) -2];
eig(A2)

g(-5-sqrt(6))
A3 = [-1 1; g(-5-sqrt(6)) -2];
eig(A3)

%%
figure(2)
clf
hold on

xoff = -5;
yoff = -5;

xeq = [0 0; -5+sqrt(6) -5+sqrt(6); -5-sqrt(6) -5-sqrt(6)];
plot(xeq(:,1),xeq(:,2),'ko')

N = 12;
R = 5;
w = linspace(0,2*pi,N+1);
x0 = [xoff+R*cos(w); yoff+R*sin(w)];
tspan = 0:0.1:5;

for i = 1:N
    [t,x] = ode45(@sys1b,tspan,x0(:,i));
    plot(x(:,1),x(:,2),'b.-')
end

[X1,X2] = meshgrid(xoff+(-R-1:R+1),yoff+(-R-1:R+1));
X1dot = zeros(size(X1));
X2dot = zeros(size(X2));
for i = 1:numel(X1)
    dxdt = sys1b(0,[X1(i) X2(i)]);
    X1dot(i) = dxdt(1);
    X2dot(i) = dxdt(2);
end
quiver(X1,X2,X1dot,X2dot,'r')

hold off
axis([xoff-R-1 xoff+R+1 yoff-R-1 yoff+R+1])
axis square

%% Problem 1(c)
figure(3)
clf
hold on

xeq = [0 0];
plot(xeq(:,1),xeq(:,2),'ko')

N = 12;
R = 2;
w = linspace(0,2*pi,N+1);
x0 = [R*cos(w); R*sin(w)];
tspan = 0:0.1:5;

for i = 1:N
    [t,x] = ode45(@sys1c,tspan,x0(:,i));
    plot(x(:,1),x(:,2),'b.-')
end

[X1,X2] = meshgrid(-R-1:R+1,-R-1:R+1);
X1dot = zeros(size(X1));
X2dot = zeros(size(X2));
for i = 1:numel(X1)
    dxdt = sys1c(0,[X1(i) X2(i)]);
    X1dot(i) = dxdt(1);
    X2dot(i) = dxdt(2);
end
quiver(X1,X2,X1dot,X2dot,'r')

hold off
axis([-R-2 R+2 -R-2 R+2])
axis square

%% Problem 1(d)
figure(4)
clf
hold on

xeq = [0 0; 1 1; -1 -1];
plot(xeq(:,1),xeq(:,2),'ko')

N = 12;
R = 5;
w = linspace(0,2*pi,N+1);
x0 = [R*cos(w); R*sin(w)];
tspan = 0:0.1:5;

for i = 1:N
    [t,x] = ode45(@sys1d,tspan,x0(:,i));
    plot(x(:,1),x(:,2),'b.-')
end

[X1,X2] = meshgrid(-R-1:R+1,-R-1:R+1);
X1dot = zeros(size(X1));
X2dot = zeros(size(X2));
for i = 1:numel(X1)
    dxdt = sys1d(0,[X1(i) X2(i)]);
    X1dot(i) = dxdt(1);
    X2dot(i) = dxdt(2);
end
quiver(X1,X2,X1dot,X2dot,'r')

hold off
axis([-R-2 R+2 -R-2 R+2])
axis square

%% Problem 2(a)
figure(5)
clf
hold on

xeq = [0 0];
plot(xeq(:,1),xeq(:,2),'ko')

N = 12;
R = 5;
w = linspace(0,2*pi,N+1);
x0 = [R*cos(w) R/2*cos(w); R*sin(w) R/2*sin(w)];
tspan = 0:0.1:5;

for i = 1:length(x0)
    [t,x] = ode45(@sys2a,tspan,x0(:,i));
    plot(x(:,1),x(:,2),'b.-')
end

[X1,X2] = meshgrid(-R-1:R+1,-R-1:R+1);
X1dot = zeros(size(X1));
X2dot = zeros(size(X2));
for i = 1:numel(X1)
    dxdt = sys2a(0,[X1(i) X2(i)]);
    X1dot(i) = dxdt(1);
    X2dot(i) = dxdt(2);
end
quiver(X1,X2,X1dot,X2dot,'r')

hold off
axis([-R-2 R+2 -R-2 R+2])
axis square

%% Problem 2(b)
figure(6)
clf
hold on

N = 24;
R = 6;
w = linspace(0,2*pi,N+1);
x0 = [R*cos(w); R*sin(w)];
tspan = 0:0.1:5;

for i = 1:length(x0)
    [t,x] = ode45(@sys2b,tspan,x0(:,i));
    plot(x(:,1),x(:,2),'b.-')
end

[X1,X2] = meshgrid(-R-1:R+1,-R-1:R+1);
X1dot = zeros(size(X1));
X2dot = zeros(size(X2));
for i = 1:numel(X1)
    dxdt = sys2b(0,[X1(i) X2(i)]);
    X1dot(i) = dxdt(1);
    X2dot(i) = dxdt(2);
end
quiver(X1,X2,X1dot,X2dot,'r')

hold off
axis([-R-2 R+2 -R-2 R+2])
axis square

%% Problem 2(c)
figure(7)
clf
hold on

N = 12;
R = 5;
w = linspace(0,2*pi,N+1);
x0 = [R*cos(w); R*sin(w)];
tspan = 0:0.1:5;

for i = 1:length(x0)
    [t,x] = ode45(@sys2c,tspan,x0(:,i));
    plot(x(:,1),x(:,2),'b.-')
end

[X1,X2] = meshgrid(-R-1:R+1,-R-1:R+1);
X1dot = zeros(size(X1));
X2dot = zeros(size(X2));
for i = 1:numel(X1)
    dxdt = sys2c(0,[X1(i) X2(i)]);
    X1dot(i) = dxdt(1);
    X2dot(i) = dxdt(2);
end
quiver(X1,X2,X1dot,X2dot,'r')

hold off
axis([-R-2 R+2 -R-2 R+2])
axis square

%% Problem 2(d)
figure(8)
clf
hold on

N = 12;
R = 1;
w = linspace(0,2*pi,N+1);
x0 = [R*cos(w) R/2*cos(w); R*sin(w) R/2*sin(w)];
tspan = 0:0.01:5;

for i = 1:length(x0)
    [t,x] = ode45(@sys2d,tspan,x0(:,i));
    plot(x(:,1),x(:,2),'b.-')
end

[X1,X2] = meshgrid(-R-1:R+1,-R-1:R+1);
X1dot = zeros(size(X1));
X2dot = zeros(size(X2));
for i = 1:numel(X1)
    dxdt = sys2d(0,[X1(i) X2(i)]);
    X1dot(i) = dxdt(1);
    X2dot(i) = dxdt(2);
end
quiver(X1,X2,X1dot,X2dot,'r')

hold off
axis([-R-2 R+2 -R-2 R+2])
axis square