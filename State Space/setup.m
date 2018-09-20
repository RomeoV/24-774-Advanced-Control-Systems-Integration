clear;
close all;

quanser_aero_parameters
quanser_aero_state_space
sys = ss(A,B,C,D);
theta = 0;
psi = 0;
theta_dot = 0;
psi_dot = 0;
figure
subplot(221)
margin(sys(1,1))
subplot(222)
margin(sys(1,2))
subplot(223)
margin(sys(2,1))
subplot(224)
margin(sys(2,2))