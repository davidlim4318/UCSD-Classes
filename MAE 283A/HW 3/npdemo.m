% Demo for Noise Prediction 
% Matlab portion of demo for MAE283a (2024)
% Written by R.A. de Callafon, Dept. of MAE, UCSD <callafon@ucsd.edu>

% Legal Dogma
% * This software is part of the course MAE283a: "Parametric System Identification - 
%   theory and tools (Fall, 2024)", or the "MAE283A 2024 course".
% * Use this software to study and understand the main computational tools 
%   behind the system identification techniques used in the MAE283A course only.
% * Feel free to re-use some of the code for your final project in the MAE283a course
% * UNAUTHORIZED COPYING, REPRODUCTION, REPUBLISHING, UPLOADING, POSTING, TRANSMITTING 
%   OR DUPLICATING OF ANY OF THE CODE BELOW OTHER THAN FOR THE PURPOSE OF
%   THE MAE283A COURSE IS PROHIBITED

% creation of orginal noise filter listed below
%wn=1;b=0.1;num=wn^2;den=[1 2*b*wn wn^2];F=tf(num,den);
%H1=c2d(F,0.1,'foh');
%[num,den]=butter(2,0.1,'high');
%num=[0.9 -1.6 0.7];
%H2=tf(num,den,0.1);
%H0=H1*H2;
%[num,den]=tfdata(H0,'v');
%num=polystab(num);
%num=num/num(1);

num=[1 -0.1];
den=[1 -0.9];

disp('Transfer function of orginal noise filter')
H0=tf(num,den,1)

% figure(1)
% bodemag(H0)
% title('Amplitude Bode response of noise filter')

% creating noise
N=500;
M=1000;
e=1*randn(N+M,1);
v=lsim(H0,e);
v=v(N+1:end);
% figure(2),figure(gcf)
% plot(v,'x-',LineWidth=2)
% xlabel('samples');
% title('observed noise signal')

% creating 1-step ahead prediction based on actual noise model knowledge
F1=(1-(1+tf(0.8,[1 0],1))*H0^-1);
F2=(1-H0^-1)^2;
% F1=(1-(1+tf(0.8,[1 0],1)+tf(0.72,[1 0 0],1)+tf(0.648,[1 0 0 0],1)+tf(0.5832,[1 0 0 0 0],1))*H0^-1);
% F2=(1-H0^-1)^5;
vp1=lsim(F1,v);
vp2=lsim(F2,v);
figure(3),figure(gcf)
plot(1:M,vp1,'o-',1:M,vp2,'*-',1:M,v,'x-',LineWidth=2)
legend('predicted 1','predicted 2','observed');
xlabel('samples');
title('observed and one-step ahead predicted noise signal')

figure(4),figure(gcf)
plot(1:M,v-vp1,'o-',1:M,v-vp2,'*-',LineWidth=2)
legend('error 1','error 2');
xlabel('samples');
title('observed and one-step ahead predicted noise signal')

var(v-vp1) % expected value is 1.64
var(v-vp2) % expected value is 1.66
