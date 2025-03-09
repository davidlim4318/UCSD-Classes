function dxdt = sys2c(t,x)
dxdt = zeros(2,1);
dxdt(1) = x(2);
dxdt(2) = -(0.5*x(1) + x(1)^3);
end