function [theta_dot_list,Xe_dt_total,Xe,Je] = FeedbackControl(X,Xd,Xdnext,Kp,Ki,dt,Xe_dt_total,theta_list)
%FEEDBACKCONTROL    Control law calculator for task-space motion control
%                   with velocity inputs for the KUKA youBot.
%   
%   [theta_dot_list,Xe_dt_total,Xe,Je] = 
%   FeedbackControl(X,Xd,Xdnext,Kp,Ki,dt,Xe_dt_total,theta_list) produces
%   a column vector (theta_dot_list) of velocity inputs for the joints and
%   wheels of the KUKA youBot to move its end-effector to the desired
%   configuration using a proportional-integral (PI) controller with a
%   feedforward component.
%
%   Input       Value
%   X           The current SE(3) configuration of the end-effector in the
%               space frame.
%   Xd          The desired SE(3) configuration of the end-effector in the
%               space frame.
%   Xdnext      The next desired SE(3) configuration of the end-effector in
%               the space frame.
%   Kp          The proportional gain.
%   Ki          The integral gain.
%   dt          The time step duration in seconds.
%   Xe_dt_total A 6-element column vector representing the time-integrated
%               error twist.
%   theta_list  A 12-element column vector containing the 3 chassis
%               configuration variables (phi,x,y), 5 arm joint angles, and
%               4 wheel angles of the KUKA youBot of the current time step.
%               Angles are in radians, x and y are in meters. Note: the
%               wheel angles are optional.
%
%   Output          Value
%   theta_dot_list  A 9-element column vector containing the 5 arm joint
%                   velocities and 4 wheel velocities calculated according
%                   to a feedforward+PI control law. The velocities are in
%                   radians/second.
%   Xe_dt_total     The updated time-integrated error twist.
%   Xe              A 6-element column vector representing the current
%                   error twist.
%   Je              A 6-by-9 matrix representing the end-effector body 
%                   Jacobian.
%
%   See also WRAPPER.

%   Written by David Lim for the MAE 204 Final Project in WI25.
%   Last modififed on 03/08/25.

Xe = se3ToVec(MatrixLog6(X\Xd));  % error twist in end-effector frame
Vd = se3ToVec(MatrixLog6(Xd\Xdnext))/dt;  % twist between the current and next configuration in space frame
Xe_dt_total = Xe*dt + Xe_dt_total;  % increment time-integrated error twist
V = Adjoint(X\Xd)*Vd + Kp*Xe + Ki*Xe_dt_total;  % twist command in end-effector frame

% compute Jacobian contribution from arm
B1 = [0 0 1 0 0.033 0]';
B2 = [0 -1 0 -0.2176-0.135-0.155 0 0]';
B3 = [0 -1 0 -0.2176-0.135 0 0]';
B4 = [0 -1 0 -0.2176 0 0]';
B5 = [0 0 1 0 0 0]';
B_list = [B1 B2 B3 B4 B5];
Jarm = JacobianBody(B_list,theta_list(4:8));

% compute SE(3) configuration of chassis in space frame
phi = theta_list(1);
x = theta_list(2);
y = theta_list(3);
Tsb = [cos(phi) -sin(phi) 0 x;
       sin(phi) cos(phi) 0 y;
       0 0 1 0.0963;
       0 0 0 1];

% compute SE(3) end-effector configuration in chassis frame
Tbe = Tsb\X;

% compute Jacobian contribution from chassis
l = 0.47/2;
w = 0.3/2;
r = 0.0475;
F = r/4*[-1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w);
         1 1 1 1;
         -1 1 -1 1];
F6 = [zeros(2,4); F; zeros(1,4)];
Jbase = Adjoint(Tbe\eye(4))*F6;

% transform twist command into joint commands
Je = [Jarm Jbase];
theta_dot_list = pinv(Je,1e-2)*V;  % TOL avoids "jerkiness" near singularities
end