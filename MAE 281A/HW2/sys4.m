function dxdt = sys4(t,x)
dxdt = zeros(3,1);
dxdt(1) = -x(1) + x(2)*x(1)*sin(x(1));
dxdt(2) = -x(2) + x(3)*x(2)*sin(x(2));
dxdt(3) = -x(3);
end