
% Specify values of system


% Constrained case
Td = 0.5920;
zeta = 8.0709e-4;
% Unconstrained case
Td = 0.46;
zeta = 0.3634;


K = exp(-zeta*pi/sqrt(1-zeta^2));
A0 = 1/(1+2*K+K^2);
A1 = 2*K/(1+2*K+K^2);
A2 = K^2/(1+2*K+K^2);
t1 = Td/2;
t2 = Td;