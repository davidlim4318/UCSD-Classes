%% Least squares estimation in frequency domain (curve fitting)

clear
load("Getfe.mat")
w = linspace(0,pi,length(P))';

m = 2000;
Gest = P(1:m);
w = w(1:m);

N = length(Gest);
% Ts = 0.00885;

loglog(w,abs(Gest))

%% 
g = ifft(Gest);  % complex-valued?
N1 = 30;
H = hankel(g(2:N1+1),g(N1+1:2*N1));  % only using 60 points?
[U,S,V] = svd(H);

figure(1)
sigma = diag(S);
plot(sigma(1:N1),'*',LineWidth=1,Color='r');
xlabel('index')
ylabel('magnitude')
set(gca,'FontSize',14)
title('Singular Values of Hankel Matrix R','FontWeight','Normal','FontSize',18)

figure(2)
sigma = diag(S);
semilogy(sigma(1:N1),'*',LineWidth=1,Color='r');
xlabel('index')
ylabel('magnitude')
set(gca,'FontSize',14)
title('Singular Values of Hankel Matrix R','FontWeight','Normal','FontSize',18)

%%
nb = 5;
na = 3;
nd = 0;
ntotal = nb+na;

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

while max_par_diff > 1e-8 && counter < 100
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

zero(G)
pole(G)

num = cell2mat(G.Numerator);
den = cell2mat(G.Denominator);

% %%
% num = deconv(cell2mat(G.Numerator),[1 -1]);
% num = conv(num,[1 -.99]);
% den = deconv(cell2mat(G.Denominator),[1 -1]);
% den = conv(den,[1 -.99]);
% G1 = tf(num,den,1);
% 
% [zero(G) zero(G1)]
% [pole(G) pole(G1)]

%%
% G1 = tf(num,den,1)*tf([1 0],[1 -1],1);
G1 = tf(num,den,1)*tf(1,[1 -1],1);
figure(7)
bode(G1)

%%
save(strcat("G","a",num2str(na),"b",num2str(nb),"d",num2str(nd),"i",num2str(counter),".mat"),"G")
save(strcat("G1","a",num2str(na),"b",num2str(nb),"d",num2str(nd),"i",num2str(counter),".mat"),"G1")





%% Constrained LS ?!?!
nb = 5;
na = 3;
nd = 0;
ntotal = nb+na;

gamma = 1.78e2;
A = [ones(1,nb) -gamma*ones(1,na)];

X = zeros(N,ntotal);
for k = nd:nd+nb-1
    X(:,1+k) = exp(-k*1j*w);
end
for k = 1:na
    X(:,nb+k) = -Gest.*exp(-k*1j*w);
end

PHI = [real(X);imag(X)];
Y = [real(Gest);imag(Gest)];

theta = [PHI'*PHI, A'; A, zeros(size(A,1))] \ [PHI' * Y; gamma];

G = tf(theta(1:nb)',conv([1 zeros(1,nd)],[1 theta(nb+1:ntotal)']),1);

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

while max_par_diff > 1e-8 && counter < 100
    X = zeros(N,na+1);
    for k = 0:na
        X(:,1+k) = exp(-k*1j*w);
    end
    Weight = X*[1;theta(nb+1:ntotal)];
    X = zeros(N,ntotal);
    for k = nd:nd+nb-1
        X(:,1+k) = exp(-k*1j*w);
    end
    for k = 1:na
        X(:,nb+k) = -Gest.*exp(-k*1j*w);
    end
    PHI = [real(Weight.\X);imag(Weight.\X)];
    Y = [real(Weight.\Gest);imag(Weight.\Gest)];
    theta_new = [PHI'*PHI, A'; A, zeros(size(A,1))] \ [PHI' * Y; gamma];
    max_par_diff = max(abs(theta - theta_new));
    theta = theta_new;
    counter = counter + 1;
    disp([num2str(counter) ': max. par. difference = ' num2str(max_par_diff) '.']);
end

G = tf(theta(1:nb)',conv([1 zeros(1,nd)],[1 theta(nb+1:ntotal)']),1);

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

zero(G)
pole(G)

num = cell2mat(G.Numerator);
den = cell2mat(G.Denominator);

% %%
% num = deconv(cell2mat(G.Numerator),[1 -1]);
% num = conv(num,[1 -.99]);
% den = deconv(cell2mat(G.Denominator),[1 -1]);
% den = conv(den,[1 -.99]);
% G1 = tf(num,den,1);
% 
% [zero(G) zero(G1)]
% [pole(G) pole(G1)]

%%
% G1 = tf(num,den,1)*tf([1 0],[1 -1],1);
G1 = tf(num,den,1)*tf(1,[1 -1],1);
figure(7)
bode(G1)

%%
save(strcat("G","a",num2str(na),"b",num2str(nb),"d",num2str(nd),"i",num2str(counter),"CLS.mat"),"G")
save(strcat("G1","a",num2str(na),"b",num2str(nb),"d",num2str(nd),"i",num2str(counter),"CLS.mat"),"G1")