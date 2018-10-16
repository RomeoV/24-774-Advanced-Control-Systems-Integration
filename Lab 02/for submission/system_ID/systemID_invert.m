%%
clear;
close all;
clc;
load('input_response.mat')
%%
A = ...
[0 0 1.0000 0
0 0 0 1.0000
0 149.2751 -0.0104 0
0 261.6091 -0.0103 0];

B = ...
[0
0
49.7275
49.1493];


Q = diag([5,20,0.05,0.05]);
R = 0.1
lqr_k = lqr(A,B,Q,R)


% Differential observer design
s = tf('s');
w_pass = 50;
obs = [1 0;
       0 1;
       w_pass*s/(s + w_pass) 0;
       0     w_pass*s/(s + w_pass)];
high_pass = w_pass*s/(s + w_pass);

% The chirp signal parameters(HZ)
f_0 = 0.1;
T = 100;
f_end = 50;
gain = 1;  

%%
qc_build_model('System_inverted');
qc_connect_model('System_inverted');
qc_start_model('System_inverted');

% pause(52);
%%
A = ...
[0 0 1.0000 0
0 0 0 1.0000
0 149.2751 -0.0104 0;
0 261.6091 -0.0103 0];

B = ...
[0
0
49.7275
49.1493];

C = [1 0 0 0;
    0 1 0 0];
D = 0;
Q = diag([5,20,0.05,0.05]);
R = 0.1
lqr_k = lqr(A,B,Q,R)


%% Differential observer design
s = tf('s');
w_pass = 50;
obs = [1 0;
       0 1;
       w_pass*s/(s + w_pass) 0;
       0     w_pass*s/(s + w_pass)];
high_pass = w_pass*s/(s + w_pass);

% The chirp signal parameters(HZ)
f_0 = 0.1;
T = 100;
f_end = 50;
gain = 1;  
%%
f_0 = 0.1;
T = 100;
f_end = 50;
gain = 1; 
Ts = 0.002;
Fs = 1/Ts;            % Sampling frequency
L = length(simout(:,1)); % Length of signal

% Extract Info from Simulink
U = fft(simout(1:end,1)/L);
U = U(1:L/2+1);
U(2:end-1) = 2 * U(2:end-1);
Y_motor = fft(simout(1:end,2)/L);
Y_motor = Y_motor(1:L/2+1);
Y_motor(2:end-1) = 2 * Y_motor(2:end-1);
Y_pen = fft(simout(1:end,3)/L);
Y_pen = Y_pen(1:L/2+1);
Y_pen(2:end-1) = 2 * Y_pen(2:end-1);


P1 = Y_motor./U;
P3 = Y_pen./U;
% Make it one sided fft
% P2 = (Y_motor./U);
% P1 = P2(1:L/2+1);
% 
% P4 = Y_pen./U;
% P3 = P4(1:L/2+1);

f = 2*pi*Fs*(0:L/2)/L;
G_1 = frd(P1,f);
G_2 = frd(P3,f);
%% Try to cut down the frequency and then fit for that
% G_1 = frd(P1(1:floor(L/2)),f(1:floor(L/2)));
% wt = makeweight(2,1.1,.01,1/Fs);
fit_1 = fitfrd(G_1,4);
% fit_half = fitmagfrd(G_1,10,2,wt);
% figure(3);
% bode(G_1,fit_1,{f_0*2*pi,f_end*2*pi})
fit_2 = fitfrd(G_2,4);
% fit_half = fitmagfrd(G_1,10,2,wt);
% figure(4);
% bode(G_2,fit_2,{f_0*2*pi,f_end*2*pi})
fit_1 = minreal(fit_1);
fit_2 = minreal(fit_2);
%%
s = tf('s');
C_lqr = lqr_k*[1 0;0 1;50*s/(s+50) 0;0 50*s/(s+50)];
real_1 = 1/(1/G_1 - C_lqr(1,1));
real_2 = 1/(1/G_2 - C_lqr(1,2));
P1 = inv(inv(tf(fit_1))-C_lqr(1,1));
P2 = inv(inv(tf(fit_2))-C_lqr(1,2));
G_ideal = ss(A,B,C,D); 
% bode([P1;P2],G_ideal,{0.1*2*pi,50*2*pi})

figure(1)
bodemag(P1, G_ideal(1),real_1,{0.1*2*pi,50*2*pi});
set(findall(gcf,'type','line'),'LineWidth',3)
% set(findall(gca,'type','plane'),'LineWidth',2)
set(gca,'FontSize',20)
set(gca,'XColor','k')
set(gca,'YColor','k')
set(gca,'LineWidth',3)
% set(findall(gcf,'type','title'),'font',20)
title('\fontsize{30}{Bode Plot for Motor}')
xlabel('\fontsize{20}{Frequency (rad/s)}')
ylabel('\fontsize{20}{Magnitude (dB)}')
legend('\fontsize{20}{Fitted Model}','\fontsize{20}{Ideal Model}','\fontsize{20}{Measured Model}');
grid on;
figure(2)
bodemag(P2,G_ideal(2),real_2,{0.1*2*pi,50*2*pi});
set(findall(gcf,'type','line'),'LineWidth',3)
% set(findall(gca,'type','plane'),'LineWidth',2)
set(gca,'FontSize',20)
set(gca,'XColor','k')
set(gca,'YColor','k')
set(gca,'LineWidth',3)
% set(findall(gcf,'type','title'),'font',20)
title('\fontsize{30}{Bode Plot for Pendulum}')
xlabel('\fontsize{20}{Frequency (rad/s)}')
ylabel('\fontsize{20}{Magnitude (dB)}')
legend('\fontsize{20}{Fitted Model}','\fontsize{20}{Ideal Model}','\fontsize{20}{Measured Model}');
grid on;