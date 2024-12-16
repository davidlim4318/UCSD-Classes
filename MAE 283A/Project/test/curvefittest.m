%% Least squares estimation in frequency domain (curve fitting)

clear
load("Getfe.mat")
w = linspace(0,pi,length(P))';

% m = length(P);
m = 1200;
Gest = P(1:m);
w = w(1:m);

N = length(Gest);
Ts = 0.00885;

loglog(w,abs(Gest))

%% 
g = ifft(Gest);
N1 = 30;
H = hankel(g(2:N1+1),g(N1+1:2*N1));
[U,S,V] = svd(H);

figure(1)
sigma = diag(S);
plot(sigma(1:N1),'*',LineWidth=1,Color='r');
xlabel('index')
ylabel('magnitude')
set(gca,'FontSize',14)
title('Singular Values of Hankel Matrix R','FontWeight','Normal','FontSize',18)

%%
nb = 4;
na = 3;
nd = 0;
ntotal = nb+na-nd;

X = zeros(N,ntotal);
for k = nd:nd+nb-1
    X(:,1+k) = exp(-k*1j*w);
end
for k = 1:na
    X(:,nb+k) = -Gest.*exp(-k*1j*w);
end

theta = [real(X);imag(X)]\[real(Gest);imag(Gest)];

G = tf(theta(1:nb)',conv([1 zeros(1,nd)],[1 theta(nb+1:end)']),1);

counter = 0;
max_par_diff = 1;

% Plot results

Gfr = reshape(freqresp(G,exp(1j*w)),N,1);

figure(3)
p3 = loglog(w,[abs(Gest) abs(Gfr)],LineWidth=2);
p3(1).Color = 'r';
p3(2).Color = 'b';
p3(2).LineStyle = ':';
xlabel('frequency (rad/s)')
ylabel('magnitude')
legend('measured','modeled')
set(gca,'FontSize',14)
title('Magnitude of the Frequency Response of the System','FontWeight','Normal','FontSize',18)
subtitle(['nb = ' num2str(nb) ', na = ' num2str(na) ', nd = ' num2str(nd) ', iteration: ' num2str(counter)])

figure(4)
p4 = semilogx(w,[rad2deg(unwrap(angle(Gest))) rad2deg(unwrap(angle(Gfr)))],LineWidth=2);
p4(1).Color = 'r';
p4(2).Color = 'b';
p4(2).LineStyle = ':';
xlabel('frequency (rad/s)')
ylabel('phase (deg)')
legend('measured','modeled')
set(gca,'FontSize',14)
title('Phase of the Frequency Response the System','FontWeight','Normal','FontSize',18)
subtitle(['nb = ' num2str(nb) ', na = ' num2str(na) ', nd = ' num2str(nd) ', iteration: ' num2str(counter)])

%% Multiple iterations with adjusted weighting

while max_par_diff > 1e-8 && counter < 10
    X = zeros(N,na+1);
    for k = 0:na
        X(:,1+k) = exp(-k*1j*w);
    end
    Weight = X*[1;theta(nb+1:end)];
    X = zeros(N,ntotal);
    for k = nd:nd+nb-1
        X(:,1+k) = exp(-k*1j*w);
    end
    for k = 1:na
        X(:,nb+k) = -Gest.*exp(-k*1j*w);
    end
    theta_new = [real(Weight.\X);imag(Weight.\X)]\[real(Weight.\Gest);imag(Weight.\Gest)];
    max_par_diff = max(abs(theta - theta_new));
    theta = theta_new;
    counter = counter + 1;
    disp([num2str(counter) ': max. par. difference = ' num2str(max_par_diff) '.']);
end

G = tf(theta(1:nb)',conv([1 zeros(1,nd)],[1 theta(nb+1:end)']),1);

% Plot results of iteration

Gfr = reshape(freqresp(G,exp(1j*w)),N,1);

figure(5)
p5 = loglog(w,[abs(Gest) abs(Gfr)],LineWidth=2);
p5(1).Color = 'r';
p5(2).Color = 'b';
p5(2).LineStyle = ':';
xlabel('frequency (rad/s)')
ylabel('magnitude')
legend('measured','modeled')
set(gca,'FontSize',14)
title('Magnitude of the Frequency Response of the System','FontWeight','Normal','FontSize',18)
subtitle(['nb = ' num2str(nb) ', na = ' num2str(na) ', nd = ' num2str(nd) ', iteration: ' num2str(counter)])

figure(6)
p6 = semilogx(w,[rad2deg(unwrap(angle(Gest))) rad2deg(unwrap(angle(Gfr)))],LineWidth=2);
p6(1).Color = 'r';
p6(2).Color = 'b';
p6(2).LineStyle = ':';
xlabel('frequency (rad/s)')
ylabel('phase (deg)')
legend('measured','modeled')
set(gca,'FontSize',14)
title('Phase of the Frequency Response the System','FontWeight','Normal','FontSize',18)
subtitle(['nb = ' num2str(nb) ', na = ' num2str(na) ', nd = ' num2str(nd) ', iteration: ' num2str(counter)])

%%
G1 = G*tf([1 0],[1 -1],1);
figure(7)
bode(G0)
hold on
bode(G1)
hold off

%%
figure(8)
impulse(G0,1000);
hold on
impulse(G1,1000)
hold off

%%
save("G1.mat","G1");