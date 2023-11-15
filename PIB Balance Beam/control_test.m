m = 2.7*10^-3;
r = 40;
I = 2/3*m*r^2;
g = 9.8*10^3;
c = 0; % need to measure
h = 0.03; % need to measure
d = 7*h; % need to measure
a_1 = c/(m + I/r^2);
b_0 = (g*m)/(m + I/r^2);

t_f = 5;
dt = h/20;

t_f = ceil((t_f+d)/h)*h;
n_f = round(t_f/dt + 1);
k_f = round(t_f/h + 1);
n_delay = d/dt;

x_0 = 0;
x = [x_0 zeros(1,n_f-1)];
dxdt = zeros(1,n_f);

x_r = 150*ones(1,n_f);

e = [x_r(1)-x_0 x_r(1)-x_0 zeros(1,k_f-2)];

theta_0 = 0;
theta = [theta_0 theta_0 zeros(1,k_f-2)];

n = 0;
k = 0;
for t = 0:dt:t_f-dt
    n = n + 1;
    if rem(t,h) == 0
        k = k + 1;
        if k > 1
            if n > n_delay
                e(k) = x_r(n-n_delay)-x(n-n_delay);
            else
                e(k) = x_r(1)-x_0;
            end
            %theta(k+1) = 0.01*e(k);
            %theta(k+1) = 0.04*e(k) + 0.032*(e(k)-e(k-1))/h;
            theta(k+1) = 0.0586737*e(k) - 0.0585857*e(k-1) + 1.1*theta(k) - 0.28*theta(k-1);
            theta(k+1) = max(-45,min(45,theta(k+1)));
        end
    end
    d2xdt2 = b_0*sin(pi*theta(k)/180);
    dxdt(n+1) = dxdt(n) + d2xdt2*dt;
    x(n+1) = x(n) + dxdt(n)*dt;
    x(n+1) = max(0,min(300,x(n+1)));
end

figure(7)
plot(0:h:h*(length(theta)-1),theta)

figure(8)
plot(0:dt:dt*(length(x)-1),x)

shg