function dxdt = sys1d(t,x)
dxdt = zeros(2,1);
dxdt(1) = -x(1)^3 + x(2);
dxdt(2) = x(1) - x(2)^3;
end