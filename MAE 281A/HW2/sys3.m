function dxdt = sys3a(t,x,c,m)
dxdt = zeros(2,1);
dxdt(1) = -c*x(1) + x(2)^(2*m)*x(1)*cos(x(1))^2;
dxdt(2) = -x(2)^3;
end