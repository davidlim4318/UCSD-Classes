%WRAPPER  Simulates pick-and-place task with the KUKA youBot.
%   
%   WRAPPER uses TRAJECTORYGENERATOR to generate a reference trajectory for
%   the end-effector, NEXTSTATE to simulate the robot motion, and
%   FEEDBACKCONTROL to compute control inputs to achieve the desired
%   motion.
%   
%   WRAPPER saves the configuration of the robot at each time step and
%   writes the data to a csv file to be visualized in CoppeliaSim Scene 6
%   associated with the book "Modern Robotics: Mechanics, Planning, and
%   Control," Kevin Lynch and Frank Park, Cambridge University Press, 2017.
%
%   WRAPPER also plots the end-effector error twist and the angular and
%   linear velocity manipulability factors over time.
%
%   See also TRAJECTORYGENERATOR, NEXTSTATE, FEEDBACKCONTROL.

%   Written by David Lim for the MAE 204 Final Project in WI25.
%   Last modififed on 03/09/25.

clear
addpath /Users/davidlim/Documents/ModernRobotics/packages/MATLAB/mr;

%% Generate desired end-effector trajectory

% SE(3) configurations of block
ang_c_init = 0;  % initial angle of block (given)
x_c_init = 1;  % initial position of block (given)
y_c_init = 0;
ang_c_fin = -pi/2;  % final angle of block (given)
x_c_fin = 0;  % final position of block (given)
y_c_fin = -1;
Tsc_initial = [cos(ang_c_init) -sin(ang_c_init) 0 x_c_init;
               sin(ang_c_init) cos(ang_c_init) 0 y_c_init;
               0 0 1 0;
               0 0 0 1];
Tsc_final = [cos(ang_c_fin) -sin(ang_c_fin) 0 x_c_fin;
             sin(ang_c_fin) cos(ang_c_fin) 0 y_c_fin;
             0 0 1 0;
             0 0 0 1];

% SE(3) configurations of end-effector
ang_e_stan = pi/2+pi/6;  % angle of end-effector at standoff
z_e_stan = 0.2;  % height of end-effector at standoff
ang_e_grab = 3*pi/4;  % angle of end-effector for grasping
z_e_grab = 0.025;  % height of end-effector for grasping
Tse_initial = [0 0 1 0;
               0 1 0 0;
               -1 0 0 0.5;
               0 0 0 1];
Tce_standoff = [cos(ang_e_stan) 0 sin(ang_e_stan) 0;
                0 1 0 0;
                -sin(ang_e_stan) 0 cos(ang_e_stan) z_e_stan;
                0 0 0 1];
Tce_grasp = [cos(ang_e_grab) 0 sin(ang_e_grab) 0;
             0 1 0 0;
             -sin(ang_e_grab) 0 cos(ang_e_grab) z_e_grab;
             0 0 0 1];

% generate trajectory of SE(3) configurations at each time step
k = 1;
[trajectory,csv_list] = TrajectoryGenerator(Tse_initial,Tsc_initial,Tsc_final,Tce_grasp,Tce_standoff,k);

%% Compute initial end-effector configuration

% initial joint configuration of robot
theta_list = [-pi/4 0 0 ...
              0 pi/2 -pi/2 -pi/4 0 ...
              0 0 0 0]';

% compute SE(3) configuration of end-effector in base frame using FK
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
T0e = FKinBody(M0e,B_list,theta_list(4:8));

% define SE(3) configuration of base in chassis frame
Tb0 = [1 0 0 0.1662;
       0 1 0 0;
       0 0 1 0.0026;
       0 0 0 1];

% compute SE(3) configuration of chassis in space frame
phi = theta_list(1);
x = theta_list(2);
y = theta_list(3);
Tsb = [cos(phi) -sin(phi) 0 x;
       sin(phi) cos(phi) 0 y;
       0 0 1 0.0963;
       0 0 0 1];

% compute initial SE(3) configuration of end-effector in space frame
X = Tsb*Tb0*T0e;

%% Simulate robot motion with velocity control

% define proportional and integral gains
Kp = 2*eye(6);
Ki = 0*eye(6);

% define maximum joint/wheel velocity
theta_dot_max = 10*pi;

% initialize variables for main simulation loop
dt = 0.01;  % time step duration
N = length(trajectory);  % total steps
theta_array = zeros(13,N);  % array of joint configurations at every time step
theta_array(1:12,1) = theta_list;  % intial joint configuration
Xe_list = zeros(6,N);  % list of error twists
Je_list = zeros(6,9,N);  % list of Jacobians
Xe_dt_total = zeros(6,1);  % time-integrated error twist

% main simulation loop
for i = 1:N-1
    Xd = trajectory{i};  % define current desired configuration
    Xdnext = trajectory{i+1};  % define next desired configuration
    % compute velocity inputs with feedback law
    [theta_dot_list,Xe_dt_total,Xe,Je] = FeedbackControl(X,Xd,Xdnext,Kp,Ki,dt,Xe_dt_total,theta_list);
    % simulate robot motion in one time step
    theta_list = NextState(theta_list,theta_dot_list,dt,theta_dot_max);
    
    % compute new SE(3) end-effector configuration
    T0e = FKinBody(M0e,B_list,theta_list(4:8));
    phi = theta_list(1);
    x = theta_list(2);
    y = theta_list(3);
    Tsb = [cos(phi) -sin(phi) 0 x;
           sin(phi) cos(phi) 0 y;
           0 0 1 0.0963;
           0 0 0 1];
    X = Tsb*Tb0*T0e;
    
    % update data arrays
    theta_array(1:12,i+1) = theta_list;
    theta_array(13,i+1) = csv_list(i,13);
    Xe_list(:,i) = Xe;
    Je_list(:,:,i) = Je;
end
% write joint configuration array to csv file
writematrix(theta_array','sim.csv')

%% Compute manipulability factors

% separate linear and angular velocity jacobians
Je_w_list = Je_list(1:3,:,:);
Je_v_list = Je_list(4:6,:,:);

% compute manipulability factors for each time step
mu_w_list = zeros(1,N);
mu_v_list = zeros(1,N);
for i = 1:N
    mu_w_list(i) = sqrt(cond(Je_w_list(:,:,i)*Je_w_list(:,:,i)'));
    mu_v_list(i) = sqrt(cond(Je_v_list(:,:,i)*Je_v_list(:,:,i)'));
end

%% Plot results

t = 0:dt:(N-1)*dt;
figure(1)
tiledlayout(2,1)

nexttile(1)
plot(t,Xe_list,'LineWidth',2)
legend({'$X_{\rm{err},1}$','$X_{\rm{err},2}$','$X_{\rm{err},3}$',...
    '$X_{\rm{err},4}$','$X_{\rm{err},5}$','$X_{\rm{err},6}$'},'Interpreter','latex')
xlabel('time (seconds)','Interpreter','latex')
ylabel('magnitude','Interpreter','latex')
set(gca,'FontSize',24,'TickLabelInterpreter','latex')

nexttile(2)
plot(t,[mu_w_list;mu_v_list],'LineWidth',2)
legend({'$\mu_1(A_\omega)$','$\mu_1(A_v)$'},'Interpreter','latex')
xlabel('time (seconds)','Interpreter','latex')
ylabel('magnitude','Interpreter','latex')
set(gca,'FontSize',24,'TickLabelInterpreter','latex')