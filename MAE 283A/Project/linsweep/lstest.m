% Notes:
% - estimating unstable integrator system with pole = 1
%   - numerical error can make pole > 1
%   - possible fix is differentiating output ONLY
%   - pole cancellation? add pole to model later
% - output is encoder signal with a little noise
%   - based on simulated OE model, even TINY noise results in LARGE bias
%   - based on simulated ARX model, LS works better
%   - conclusion: OUTPUT noise is white, EQUATION noise is NOT white
%   - LS white equation noise requirement NOT satisfied
% - IV method
%   - works on simulation data, not on real data :(

%% Parameter estimation

% Load data
clear
% M = readmatrix("2DOF_1.txt",'Whitespace',[';','[',']']);
% y = M(:,5); % x1
% u = M(:,7);
M = readmatrix("lin_sweep.txt",'Whitespace',[';','[',']']);
y = M(:,5); % x1
u = M(:,8);

load('Ga3b4d0i15.mat')
load('G1a3b4d0i15.mat')
yn = lsim(G1,u);
y = yn + 0.1*randn(length(u),1);
% y = lsim(G1,u) + lsim(tf(1,cell2mat(G1.Denominator),1),0.1*randn(length(u),1));

m = 1000;

% Prefilter
numf = [1 -1];
denf = [1 0];
% den = 1;
uf = u(1:m);
yf = filter(numf,denf,y(1:m));

% figure(3)
% plot([uf/max(abs(uf)),yf/max(abs(yf))])
plot([filter(numf,denf,yn(1:m)) yf(1:m)])  %% NOISE IS TINY!!!

%%

% Define indices
t_i = 4;
N = length(yf);
idx = (t_i:N);

% Define data matrices
PHI = [ uf(idx) uf(idx-1) uf(idx-2) uf(idx-3) -yf(idx-1) -yf(idx-2) -yf(idx-3) ];
Y = yf(idx);

% Compute LS estimate
theta_LS = PHI\Y;

% Define discrete-time transfer function model
num = [theta_LS(1:4)]';
den = [1 theta_LS(5:end)'];
Gls = tf(num,den,1);
Gls1 = Gls*tf([1 0],[1 -1],1);

%% Simulation

% Simulate model with input data
y_sim = lsim(Gls,uf);

% Plot results
figure(1)
h = plot([yf y_sim],'LineWidth',2);
h(1).Color = 'r';
h(2).Color = 'b';
h(2).LineStyle = ':';
xlabel('time (step)')
ylabel('position (cm)')
legend('measured','simulated')
set(gca,'FontSize',14)
title('Position vs. Time of a Mass-Spring-Damper System','FontWeight','Normal','FontSize',18)

%% Simulation

% Simulate model with input data
y_sim1 = lsim(Gls1,u);

% Plot results
figure(2)
h = plot([y y_sim1],'LineWidth',2);
h(1).Color = 'r';
h(2).Color = 'b';
h(2).LineStyle = ':';
xlabel('time (step)')
ylabel('position (cm)')
legend('measured','simulated')
set(gca,'FontSize',14)
title('Position vs. Time of a Mass-Spring-Damper System','FontWeight','Normal','FontSize',18)

% %%
% 
% figure(2)
% plot(y-y_sim,'LineWidth',2);
% xlabel('time (s)')
% ylabel('simulation error (cm)')
% set(gca,'FontSize',14)
% title('Position vs. Time of a Mass-Spring-Damper System','FontWeight','Normal','FontSize',18)





%% IV method (simulation data)

% Prefilter
uf = u;
yf = filter(numf,denf,y);

% Define indices
t_i = 4;
N = length(yf);
idx = (t_i:N);

% Define data matrices
PHI = [ uf(idx) uf(idx-1) uf(idx-2) uf(idx-3) -yf(idx-1) -yf(idx-2) -yf(idx-3) ];
XI = [ uf(idx) uf(idx-1) uf(idx-2) uf(idx-3) lsim(G,uf(idx)) lsim(G,uf(idx-1)) lsim(G,uf(idx-2)) ];
Y = yf(idx);

% Compute LS estimate
theta_IV = (XI'*PHI)\(XI'*Y);

% Define discrete-time transfer function model
num = [theta_IV(1:4)]';
den = [1 theta_IV(5:end)'];
Giv = tf(num,den,1);
Giv1 = Giv*tf([1 0],[1 -1],1);

%% Simulation

% Simulate model with input data
y_sim3 = lsim(Giv,uf);

% Plot results
figure(3)
h = plot([yf y_sim3],'LineWidth',2);
h(1).Color = 'r';
h(2).Color = 'b';
h(2).LineStyle = ':';
xlabel('time (step)')
ylabel('position (cm)')
legend('measured','simulated')
set(gca,'FontSize',14)
title('Position vs. Time of a Mass-Spring-Damper System','FontWeight','Normal','FontSize',18)

%% Simulation

% Simulate model with input data
y_sim4 = lsim(Giv1,u);

% Plot results
figure(4)
h = plot([y y_sim4],'LineWidth',2);
h(1).Color = 'r';
h(2).Color = 'b';
h(2).LineStyle = ':';
xlabel('time (step)')
ylabel('position (cm)')
legend('measured','simulated')
set(gca,'FontSize',14)
title('Position vs. Time of a Mass-Spring-Damper System','FontWeight','Normal','FontSize',18)



%{
%% IV method (real data)

y = M(:,5); % x1
u = M(:,8);

% Prefilter
uf = u;
yf = filter(numf,denf,y);

% Define indices
t_i = 7;
N = length(yf);
idx = (t_i:N);

% Define data matrices
PHI = [ uf(idx) uf(idx-1) uf(idx-2) uf(idx-3) -yf(idx-1) -yf(idx-2) -yf(idx-3) ];
XI = [ uf(idx) uf(idx-1) uf(idx-2) uf(idx-3) uf(idx-4) uf(idx-5) uf(idx-6) ];
Y = yf(idx);

% Compute LS estimate
theta_IV = (XI'*PHI)\(XI'*Y);

% Define discrete-time transfer function model
num = [theta_IV(1:4)]';
den = [1 theta_IV(5:end)'];
Giv = tf(num,den,1);
Giv1 = Giv*tf([1 0],[1 -1],1);

%% Simulation

% Simulate model with input data
y_sim3 = lsim(Giv,uf);

% Plot results
figure(3)
h = plot([yf y_sim3],'LineWidth',2);
h(1).Color = 'r';
h(2).Color = 'b';
h(2).LineStyle = ':';
xlabel('time (step)')
ylabel('position (cm)')
legend('measured','simulated')
set(gca,'FontSize',14)
title('Position vs. Time of a Mass-Spring-Damper System','FontWeight','Normal','FontSize',18)

%% Simulation

% Simulate model with input data
y_sim4 = lsim(Giv1,u);

% Plot results
figure(4)
h = plot([y y_sim4],'LineWidth',2);
h(1).Color = 'r';
h(2).Color = 'b';
h(2).LineStyle = ':';
xlabel('time (step)')
ylabel('position (cm)')
legend('measured','simulated')
set(gca,'FontSize',14)
title('Position vs. Time of a Mass-Spring-Damper System','FontWeight','Normal','FontSize',18)
%}