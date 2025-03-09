%% Homework 5, Exercise 5.25

% David Lim
% A16398479
% 02/18/25

clear
% Add path to MR functions
addpath /Users/davidlim/Documents/ModernRobotics/packages/MATLAB/mr;

% Define fixed parameters
W1 = 0.109;
W2 = 0.082;
L1 = 0.425;
L2 = 0.392;
H1 = 0.089;
H2 = 0.095;

% Define variable parameters
theta1 = pi/2;
theta2 = theta1;
theta3 = theta1;
theta4 = theta1;
theta5 = theta1;
theta6 = theta1;

% Define screw axes in {s} frame
S1 = [0 0 1 0 0 0]';
S2 = [0 1 0 -H1 0 0]';
S3 = [0 1 0 -H1 0 L1]';
S4 = [0 1 0 -H1 0 L1+L2]';
S5 = [0 0 -1 -W1 L1+L2 0]';
S6 = [0 1 0 H2-H1 0 L1+L2]';

%% (a) Compute space Jacobian
Js1 = S1;
Js2 = Adjoint(MatrixExp6(VecTose3(S1*theta1)))*S2;
Js3 = Adjoint(MatrixExp6(VecTose3(S1*theta1))*...
    MatrixExp6(VecTose3(S2*theta2)))*S3;
Js4 = Adjoint(MatrixExp6(VecTose3(S1*theta1))*...
    MatrixExp6(VecTose3(S2*theta2))*...
    MatrixExp6(VecTose3(S3*theta3)))*S4;
Js5 = Adjoint(MatrixExp6(VecTose3(S1*theta1))*...
    MatrixExp6(VecTose3(S2*theta2))*...
    MatrixExp6(VecTose3(S3*theta3))*...
    MatrixExp6(VecTose3(S4*theta4)))*S5;
Js6 = Adjoint(MatrixExp6(VecTose3(S1*theta1))*...
    MatrixExp6(VecTose3(S2*theta2))*...
    MatrixExp6(VecTose3(S3*theta3))*...
    MatrixExp6(VecTose3(S4*theta4))*...
    MatrixExp6(VecTose3(S5*theta5)))*S6;

Js = [Js1 Js2 Js3 Js4 Js5 Js6];
Jw = Js(1:3,:);
Jv = Js(4:6,:);

disp(round(Js,12))
disp(' ')
disp(round(Jw,12))
disp(' ')
disp(round(Jv,12))
disp(' ')

%% (b) Compute velocity manipulability ellipsoids
Aw = Jw*Jw';
Av = Jv*Jv';

[Vw,Dw] = eig(Aw);
[Vv,Dv] = eig(Av);

disp(Vw)
disp(' ')
disp(sqrt(Dw))
disp(' ')
disp(Vv)
disp(' ')
disp(sqrt(Dv))
disp(' ')

syms qx qy qz real

figure(1)
fimplicit3([qx; qy; qz]'*Aw^-1*[qx; qy; qz]==1,'EdgeColor','none','FaceAlpha',0.5)
xlabel('$$x_s$$ (rad/s)','Interpreter','latex')
ylabel('$$y_s$$ (rad/s)','Interpreter','latex')
zlabel('$$z_s$$ (rad/s)','Interpreter','latex')
title('Angular-Velocity Manipulability Ellipsoid','Interpreter','latex')
set(gca,'FontSize',12,'TickLabelInterpreter','latex')
axis equal

figure(2)
fimplicit3([qx; qy; qz]'*Av^-1*[qx; qy; qz]==1,'EdgeColor','none','FaceAlpha',0.5)
xlabel('$$x_s$$ (m/s)','Interpreter','latex')
ylabel('$$y_s$$ (m/s)','Interpreter','latex')
zlabel('$$z_s$$ (m/s)','Interpreter','latex')
title('Linear-Velocity Manipulability Ellipsoid','Interpreter','latex')
set(gca,'FontSize',12,'TickLabelInterpreter','latex')
axis equal

%% (c) Compute force ellipsoids
Bw = Aw^-1;
Bv = Av^-1;

[Vw,Dw] = eig(Bw);
[Vv,Dv] = eig(Bv);

disp(Vw)
disp(' ')
disp(sqrt(Dw))
disp(' ')
disp(Vv)
disp(' ')
disp(sqrt(Dv))
disp(' ')

figure(3)
fimplicit3([qx; qy; qz]'*Bw^-1*[qx; qy; qz]==1,'EdgeColor','none','FaceAlpha',0.5)
xlabel('$$x_s$$ (N*m)','Interpreter','latex')
ylabel('$$y_s$$ (N*m)','Interpreter','latex')
zlabel('$$z_s$$ (N*m)','Interpreter','latex')
title('Moment Force Manipulability Ellipsoid','Interpreter','latex')
set(gca,'FontSize',12,'TickLabelInterpreter','latex')
axis equal

figure(4)
fimplicit3([qx; qy; qz]'*Bv^-1*[qx; qy; qz]==1,'EdgeColor','none','FaceAlpha',0.5)
xlabel('$$x_s$$ (N)','Interpreter','latex')
ylabel('$$y_s$$ (N)','Interpreter','latex')
zlabel('$$z_s$$ (N)','Interpreter','latex')
title('Linear Force Manipulability Ellipsoid','Interpreter','latex')
set(gca,'FontSize',12,'TickLabelInterpreter','latex')
axis equal