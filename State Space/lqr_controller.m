clear;
clc;

% load parameters
quanser_aero_parameters;
quanser_aero_state_space;
real_sys = ss(A,B,C,D);

% Define reference for simplicity
pitch_amp = pi/6;
pitch_freq = 0.4;

yaw_amp = pi/4;
yaw_freq = 0.5;


% Change the C, D matrix such that assume full state feedback for testing
% lqr controller
C_con = eye(4);
D_con = [];

control_sys = ss(A,B,C_con,D_con);

% Penalty matrix
Qc = diag([100,50,0.01,0.01]);
Rc = diag([0.01,0.01]);
K = lqr(A,B,Qc,Rc);



% Observation system using Kalman Filter
Qo = diag([0.5, 0.5]);
Ro = diag([1, 1]);
real_sys = ss(A,B,C,D);

%[K_kalman, L, P] = kalman(real_sys, Qo, Ro);

% P puts the eigenvalue  
% BEST SO FAR: P = [-120 -150 -100 -80];

P = [-120 -150 -100 -80];
L = place(A',C',P)';

% Form the observer lti system
A_obs = A - L*C;

% upper input is control, lower is sensor measurement
B_obs = [B, L];
C_obs = eye(4);
obs_controls = size(B_obs,2);
obs_sys_kalman = ss(A_obs,B_obs,C_obs,[]);


% Observation system using a derivative plus low pass filter
% [    pitch(s)
%        phi(s)  = G * [pitch(s)
%  pitch_dot(s)           phi(s)]
%    phi_dot(s)]

s = tf('s');
wcross_pitch = 10;
wcross_yaw = 10;

G = [1 0;
    0 1;
    wcross_pitch*s/(s + wcross_pitch) 0;
    0 wcross_yaw*s/(s + wcross_yaw)];
% G = [1 0;
%      0 1;
%      s 0;
%      0 s];
obs_sys_diff = ss(G);
disp('Run')













