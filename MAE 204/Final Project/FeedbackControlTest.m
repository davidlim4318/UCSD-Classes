
clear
clc

theta_list = [0 0 0 0 0 0.2 -1.6 0]';

M0e = [1 0 0 0.033;
       0 1 0 0;
       0 0 1 0.2176+0.135+0.155+0.147;
       0 0 0 1];
B1 = [0 0 1 0 0.033 0]';
B2 = [0 -1 0 -0.2176-0.135-0.155 0 0]';
B3 = [0 -1 0 -0.2176-0.135 0 0]';
B4 = [0 -1 0 -0.2176 0 0]';
B5 = [0 0 1 0 0 0]';
B_list = [B1 B2 B3 B4 B5];
T0e = FKinBody(M0e,B_list,theta_list(4:end));

Tb0 = [1 0 0 0.1662;
       0 1 0 0;
       0 0 1 0.0026;
       0 0 0 1];

phi = theta_list(1);
x = theta_list(2);
y = theta_list(3);
Tsb = [cos(phi) -sin(phi) 0 x;
       sin(phi) cos(phi) 0 y;
       0 0 1 0.0963;
       0 0 0 1];

X = Tsb*Tb0*T0e;

Xd = [0 0 1 0.5;
      0 1 0 0;
      -1 0 0 0.5;
      0 0 0 1];

Xdnext = [0 0 1 0.6;
          0 1 0 0;
          -1 0 0 0.3;
          0 0 0 1];

Kp = eye(6);

Ki = zeros(6);

dt = 0.01;

Xe_dt_total = zeros(6,1);

[V,theta_dot_list,Xe_dt_total] = FeedbackControl(X,Xd,Xdnext,Kp,Ki,dt,Xe_dt_total,theta_list)

