function dxdt = sys1c(t,x)
dxdt = zeros(2,1);
dxdt(1) = -x(1) + x(2)*(1 + x(1));
dxdt(2) = -x(1)*(1 + x(1));
end