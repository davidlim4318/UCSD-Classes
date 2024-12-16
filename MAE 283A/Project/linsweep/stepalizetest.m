%% 1. Load and organize data into Hankel matrices

clear

M = readmatrix("2DOF_1.txt",'Whitespace',[';','[',']']);
t = M(:,3);
y = 2*M(:,5); % x1
u = 2*M(:,7);

num = [1 -1];
% den = [1 0];
den = 1;
yf = filter(num,den,y(1:339));

N = length(yf);
N1 = floor(N/2);
R = hankel(yf(2:N1),yf(N1:2*N1-1)) - kron(yf(1:N1-1),ones(1,N1));
Rbar = hankel(yf(3:N1+1),yf(N1+1:2*N1)) - kron(yf(2:N1),ones(1,N1));

%% 2. Perform singular value decomposition

figure(1)
[U,S,V] = svd(R);
sigma = diag(S);
plot(sigma(1:30),'*',LineWidth=1,Color='r');
xlabel('index')
ylabel('magnitude')
set(gca,'FontSize',14)
title('Singular Values of Hankel Matrix R','FontWeight','Normal','FontSize',18)

%% 3. Approximate R with R1*R2

n = 3;
R1 = U(:,1:n)*sqrt(S(1:n,1:n));
R2 = sqrt(S(1:n,1:n))*V(:,1:n)';

% 4. Compute inverses

R1dagger = sqrt(S(1:n,1:n))\U(:,1:n)';
R2dagger = V(:,1:n)/sqrt(S(1:n,1:n));

% 5. Compute state space model

A = R1dagger*Rbar*R2dagger;
B = R2(:,1);
C = R1(1,:);
D = yf(1);

% Ts = 0.0266;
G = ss(A,B,C,D,1);
[num,den] = tfdata(G,'v');

roots(num)
roots(den)

%% Simulate and plot results
y_sim1 = step(G,N-1);

figure(2)
p2 = plot([yf y_sim1],LineWidth=2);
p2(1).Color = 'r';
p2(2).Color = 'b';
p2(2).LineStyle = ':';
xlabel('time (steps)')
ylabel('position (counts)')
legend('measured','simulated')
set(gca,'FontSize',14)
title('Position vs. Time of a Mass-Spring-Damper System','FontWeight','Normal','FontSize',18)

%%
% G1 = tf(num,den,1)*tf([1 0],[1 -1],1);
G1 = tf(num,den,1)*tf(1,[1 -1],1);
figure(7)
bode(G1)

%% Simulate and plot results
y_sim1 = lsim(G1,u);

figure(3)
p2 = plot([y y_sim1],LineWidth=2);
p2(1).Color = 'r';
p2(2).Color = 'b';
p2(2).LineStyle = ':';
xlabel('time (steps)')
ylabel('position (counts)')
legend('measured','simulated')
set(gca,'FontSize',14)
title('Position vs. Time of a Mass-Spring-Damper System','FontWeight','Normal','FontSize',18)