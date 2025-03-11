function dxdt = sys4(t,x,eps,a)
dxdt = zeros(2,1);
dxdt(1) = -x(1)*x(2);
dxdt(2) = -(x(2)-sin(x(1))^2) * (x(2)-exp(a*x(1))) * (x(2)-2*exp(2*a*x(1))) / eps;
end