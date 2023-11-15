clear
data1 = [5 7 9 11 13 15 17 19 21 23 25 27 29 31 33 35 37;
    7.5 9.5 11.8 13.8 16 17.7 19.8 21.5 23.3 24.9 26.6 28.3 29.8 31. 32.4 33.8 34.8];
y = data1(1,:)*10;
x = data1(2,:)*10;

p = polyfit(x,y,2);
yp = polyval(p,x);
clf
hold on
plot(x,y,'o')
plot(x,yp)
legend('1','2')
% plot(data1(1,:),data1(1,:) - data1(2,:))
shg