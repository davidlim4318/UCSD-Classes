clear
data1 = [5 7 9 11 13 15 17 19 21 23 25 27 29 31 33 35 37;
    7.5 9.5 11.8 13.8 16 17.7 19.8 21.5 23.3 24.9 26.6 28.3 29.8 31. 32.4 33.8 34.8];
data2 = [5 7 9 11 13 15 17 19 21 23 25 27 29 31 33 35 37;
     5 7 9 11 13 14.7 16.8 18.7 20.0 21.7 23.2 24.3 25.7 26.5 27.3 28.0 29.3];
y1 = data1(1,:)*10;
x1 = data1(2,:)*10;

p = polyfit(x1,y1,3);
yp = polyval(p,x1);
clf
hold on
plot(x1,y1,'o')
plot(x1,yp)
%legend('1','2')
% plot(data1(1,:),data1(1,:) - data1(2,:))
shg

y2 = data2(1,:)*10;
x2 = data2(2,:)*10;
p = polyfit(x2,y2,3);
yp = polyval(p,x2);
%clf
hold on
plot(x2,y2,'o')
plot(x2,yp)
legend('1','2','3','4')
% plot(data1(1,:),data1(1,:) - data1(2,:))
axis equal
shg