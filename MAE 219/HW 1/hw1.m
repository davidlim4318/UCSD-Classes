clear

m_a = 0.007;
m_h = 0.025;
k_s = 1337;
k_u = 509;
b_s = 1.1;
b_u = 8;

T1 = tf([m_a 0 0 0 0],conv([m_h b_s+b_u k_s+k_u],[m_a b_s k_s]));

m_a = 0.02;

T2 = tf([m_a 0 0 0 0],conv([m_h b_s+b_u k_s+k_u],[m_a b_s k_s]));

m_a = 0.007;
b_s = 0.02;

T3 = tf([m_a 0 0 0 0],conv([m_h b_s+b_u k_s+k_u],[m_a b_s k_s]));

b_s = 1.1;
k_u = 0;
b_u = 0;

T4 = tf([m_a 0 0 0 0],conv([m_h b_s+b_u k_s+k_u],[m_a b_s k_s]));

%%
figure(1)
clf
hold on
bode(T1)
bode(T2)
bode(T3)
bode(T4)
hold off
shg

title('Voice Coil Acutator Model Bode Plot')
legend({'reference','m_a = 0.02','b_s = 0.02','k_u = 0, b_u = 0'})