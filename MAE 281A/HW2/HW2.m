%% Problem 3a

figure(1)
clf
tiledlayout(2,1)

x0 = 0.1;
y0 = 1;
X0 = [0 0; x0 y0; -x0 y0];
N = length(X0);
tspan = 0:0.1:10;

c = 0;
m = 2;

for i = 1:N
    [t,X] = ode45(@(t,X) sys3(t,X,c,m), tspan, X0(i,:));
    nexttile(1)
    hold on
    plot(tspan,X(:,1),'-')
    nexttile(2)
    hold on
    plot(tspan,X(:,2),'-')
end

xb = abs(x0)*exp(y0^(2*(m-1))/(2*(1-m)).*(1./(1+2*y0^2*tspan).^(m-1)-1));

nexttile(1)
plot(tspan,xb,'--')
plot(tspan,-xb,'--')

%% Problem 3b

figure(2)
clf
tiledlayout(2,1)

x0 = 0.1;
y0 = 2;
X0 = [0 0; x0 y0; -x0 y0];
N = length(X0);
tspan = 0:0.1:5;

c = 1;
m = 1;

for i = 1:N
    [t,X] = ode45(@(t,X) sys3(t,X,c,m), tspan, X0(i,:));
    nexttile(1)
    hold on
    plot(tspan,X(:,1),'-')
    nexttile(2)
    hold on
    plot(tspan,X(:,2),'-')
end

xb = abs(x0)*exp(-c*tspan).*sqrt(1+2*y0^2*tspan);

nexttile(1)
plot(tspan,xb,'--')
plot(tspan,-xb,'--')

%% Problem 4

figure(3)
clf
tiledlayout(3,1)

x0 = 1;
y0 = 0.1;
z0 = 0.1;
X0 = [0 0 0; x0 y0 z0; -x0 y0 z0];
N = size(X0,1);
tspan = 0:0.1:5;

for i = 1:N
    [t,X] = ode45(@(t,X) sys4(t,X), tspan, X0(i,:));
    nexttile(1)
    hold on
    plot(tspan,X(:,1),'-')
    nexttile(2)
    hold on
    plot(tspan,X(:,2),'-')
    nexttile(3)
    hold on
    plot(tspan,X(:,3),'-')
end

xb = abs(x0)*exp(abs(y0)*exp(abs(z0)))*exp(-tspan);

nexttile(1)
plot(tspan,xb,'--')
plot(tspan,-xb,'--')