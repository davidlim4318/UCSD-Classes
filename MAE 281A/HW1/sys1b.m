function dxdt = sys1b(t,x)
dxdt = zeros(2,1);
dxdt(1) = -x(1) + x(2);
dxdt(2) = 0.1*x(1) - 2*x(2) - x(1)^2 - 0.1*x(1)^3;
end