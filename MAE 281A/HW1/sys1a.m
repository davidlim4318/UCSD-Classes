function dxdt = sys1a(t,x)
dxdt = zeros(2,1);
dxdt(1) = x(2);
dxdt(2) = -x(1) + x(1)^3/6 - x(2);
end