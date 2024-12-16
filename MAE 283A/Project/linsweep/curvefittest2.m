%% Least squares estimation in frequency domain (curve fitting)
% - added scaling to condition matrices
% - constrain a pole to 1 exactly
% - running into weighting issues
%   - freq resp goes to inf at w = 0, weighting goes to inf

clear
load("Gspa.mat")
w = linspace(0,pi,length(P))';

m = 1500;
Gspa = P(2:m);
w = w(2:m);
N = length(Gspa);

loglog(w,abs(Gspa))

%% Constrained LS ?!?!
nb = 5;
na = 4;
nd = 0;
ntotal = nb+na;

scale1 = 1e-5;

b = -1;
A = [zeros(1,nb) ones(1,na)];

X = zeros(N,ntotal);
for k = nd:nd+nb-1
    X(:,1+k) = exp(-k*1j*w);
end
for k = 1:na
    X(:,nb+k) = -Gspa*scale1.*exp(-k*1j*w);
end

PHI = [real(X);imag(X)];
Y = [real(Gspa*scale1);imag(Gspa*scale1)];

theta = [PHI'*PHI, A'; A, zeros(size(A,1))] \ [PHI' * Y; b];

theta = [theta(1:nb)/scale1; theta(nb+1:end)];
G = tf(theta(1:nb)',[1 theta(nb+1:ntotal)'],1);

counter = 0;
max_par_diff = 1;

% Plot results

Gfr = reshape(freqresp(G,exp(1j*w)),N,1);

figure(3)
p3 = loglog(w,[abs(Gspa) abs(Gfr)],LineWidth=2);
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
p4 = semilogx(w,[rad2deg(unwrap(angle(Gspa))) rad2deg(unwrap(angle(Gfr)))],LineWidth=2);
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

    hpf = freqz([1 -1],1,w);  % curb weighting function
    Weight = hpf.\X*[1;theta(nb+1:ntotal)];

    X = zeros(N,ntotal);
    for k = nd:nd+nb-1
        X(:,1+k) = exp(-k*1j*w);
    end
    for k = 1:na
        X(:,nb+k) = -Gspa*scale1.*exp(-k*1j*w);
    end
    PHI = [real(Weight.\X);imag(Weight.\X)];
    Y = [real(Weight.\Gspa*scale1);imag(Weight.\Gspa*scale1)];
    
    scale2 = 10^round(log10(max(PHI'*PHI,[],'all')));  % dynamic scaling
    b = -1*scale2;
    A = [zeros(1,nb) ones(1,na)*scale2];

    theta_new = [PHI'*PHI, A'; A, zeros(size(A,1))] \ [PHI' * Y; b];
    theta_new = [theta_new(1:nb)/scale1; theta_new(nb+1:end)];
    max_par_diff = max(abs(theta - theta_new));
    theta = theta_new;
    counter = counter + 1;
    disp([num2str(counter) ': max. par. difference = ' num2str(max_par_diff) '.']);
end

G = tf(theta(1:nb)',[1 theta(nb+1:ntotal)'],1);

% Plot results of iteration

Gfr = reshape(freqresp(G,exp(1j*w)),N,1);

figure(5)
p5 = loglog(w,[abs(Gspa) abs(Gfr)],LineWidth=2);
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
p6 = semilogx(w,[rad2deg(unwrap(angle(Gspa))) rad2deg(unwrap(angle(Gfr)))],LineWidth=2);
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

%%
figure(7)
bode(G)

%%
save(strcat("G2","a",num2str(na),"b",num2str(nb),"d",num2str(nd),"i",num2str(counter),"CLS.mat"),"G")