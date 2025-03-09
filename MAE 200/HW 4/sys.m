function dxdt = sys(t,x,m1,m2,k1,k2,c1,c2)
dxdt = zeros(4,1);
y = x(1);
yd = x(2);
z = x(3);
zd = x(4);
dxdt(1) = yd;
dxdt(2) = ( -k1*y + k2*(z-y) - c1*yd - c2*(yd-zd) ) / m1;
dxdt(3) = zd;
dxdt(4) = ( k2*(y-z) + c2*(yd-zd) ) / m2;
end