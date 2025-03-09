function dxdt = sys2d(t,x)
dxdt = zeros(2,1);
dxdt(1) = x(2);
if abs(x(1) - x(2)) <= 1
    dxdt(2) = -x(2) - ( (x(1)-x(2))^3 + 0.5*(x(1)-x(2)) );
else
    dxdt(2) = -x(2) - ( 2*(x(1)-x(2)) - 0.5 );
end