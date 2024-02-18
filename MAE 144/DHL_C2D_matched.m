function [Gz]=DHL_C2D_matched(Gs,h,omega_bar,causal)
    % function [Gz]=DHL_C2D_matched(Gs,h,omega_bar,causal)
    % Description: Compute the Gz(z) corresponding to Gs(s) using the 
    % matched method with timestep h.
    % Optional: Specify a frequency of interest omega_bar at which the
    % gain of Gz should match the gain of Gs (default is omega_bar=0).
    % Optional: Specify 'strict' for causal if Gs is to be strictly
    % causal.
    % TEST: syms s z1 p1, as = [1 p1 0]; bs = [1 z1]; Ds = RR_tf(bs,as); h = 0.1; Gz=DHL_C2D_matched(Ds,h)
    % TEST: as = [1 10 0]; bs = [1 1]; Ds = RR_tf(bs,as); h = 0.1; Gz=DHL_C2D_matched(Ds,h)
    % MAE 144 Homework 1, Problem 2a, David Lim, A16398479, 10/11/23

    % Check for number of inputs, use default values if necessary
    if nargin <= 3
        causal = 'strict';
        if nargin <= 2
            omega_bar = 0;
        end
    end

    % Determine the zeros and poles in the s-domain
    m = Gs.num.n;
    n = Gs.den.n;
    zeros_s = RR_roots(Gs.num);
    poles_s = RR_roots(Gs.den);
    
    % Step (i): Map the zeros and poles to the z-domain
    zeros_z = exp(zeros_s*h);
    poles_z = exp(poles_s*h);
    
    % Step (ii): Check for infinite zeros in s-domain, add zeros in the z-domain if necessary
    if n - m > 0
        if strcmp(causal,'strict')
            zeros_z = [zeros_z -ones(n-m-1)];
        else
            zeros_z = [zeros_z -ones(n-m)];
        end
    end

    % Define the intermediate Gz transfer function
    Gz = RR_tf(zeros_z,poles_z,1); 
    Gz.h = h;
    
    % Step (iii): Match the gains of each transfer function at frequency omega_bar
    while ismember(0,RR_roots(Gs.den)) && ismember(1,RR_roots(Gz.den))
        temp = RR_tf(RR_poly(1,1));
        temp.h = h;
        Gz = Gz*temp;
        temp = RR_tf(RR_poly(0,1));
        Gs = Gs*temp;
    end
    s_star = omega_bar*1i;
    G_z = abs(RR_evaluate(Gz,exp(s_star*h)));
    G_s = abs(RR_evaluate(Gs,s_star));
    K = G_s/G_z;
    
    % Define the final transfer function
    Gz = RR_tf(zeros_z,poles_z,K); 
    Gz.h = h;

end