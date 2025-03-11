%% Problem 4
figure(1)
clf
hold on

R = 5;
% N = 12;
% w = linspace(0,2*pi,N+1);
% x0 = [R*cos(w); R*sin(w)];
x0 = [-4.5 -4.5 -4.5 -1.5 3.5 3.5 ; 
      3.5 -3.5 0.5 0.5 3.5 -3.5];
N = size(x0,2);
tspan = 0:0.01:5;

eps = 0.01;
a = 0.25;

for i = 1:N
    [t,x] = ode45(@(t,x) sys4(t,x,eps,a),tspan,x0(:,i));
    plot(x(:,1),x(:,2),'.-','LineWidth',2,'MarkerSize',10)
end

[X1,X2] = meshgrid(-R-1:0.5:R+1,-R-1:0.5:R+1);
X1dot = zeros(size(X1));
X2dot = zeros(size(X2));
for i = 1:numel(X1)
    dxdt = sys4(0,[X1(i) X2(i)],eps,a);
    X1dot(i) = dxdt(1);
    X2dot(i) = dxdt(2);
end
V = sqrt(X1dot.^2+X2dot.^2);
X1dotn = X1dot./V;
X2dotn = X2dot./V;
quiver(X1,X2,X1dotn,X2dotn,'r')

syms x
z1 = sin(x)^2;
z2 = exp(a*x);
z3 = 2*exp(2*a*x);

fplot(z1,[-R-1 R+1])
fplot(z2,[-R-1 R+1])
fplot(z3,[-R-1 R+1])

hold off
axis square
axis([-R-2 R+2 -R-2 R+2])

