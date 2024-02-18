clear
w_c = 1;
n = 2;

%%
[lpf,hpf] = RR_complementary_filters(w_c,n);
RR_bode(lpf)
hold on
RR_bode(hpf)
hold off

%%
syms s
lpf = 1/(1+(s/w_c)^n);
lpf = simplify(lpf);
[num,den] = numden(lpf);
num = double(fliplr(coeffs(num,'all')));
den = double(fliplr(coeffs(den,'all')));

%%
lpf = RR_tf(num,den);
hpf = 1 - lpf;
clf
RR_bode(lpf)
hold on
RR_bode(hpf)
hold off

%% 
[b,a] = butter(n,w_c,'s');
lpf = tf(b,a);
[b,a] = butter(n,w_c,'high','s');
hpf = tf(b,a);
clf
bode(lpf)
hold on
bode(hpf)
bode(1 - lpf)
hold off
