%%
clear;
close all;
clc;

% The chirp signal parameters(HZ)
f_0 = 0.1;
T = 50;
f_end = 20;
gain = 0.5;  


qc_build_model('sysID_Simulink');
qc_connect_model('sysID_Simulink');
qc_start_model('sysID_Simulink');

pause(52);
%%

Ts = 0.001;
Fs = 1/Ts;            % Sampling frequency
L = length(simout(:,1))-1; % Length of signal

% Extract Info from Simulink
U = fft(simout(1:end-1,1)/L);
U = U(1:L/2+1);
U(2:end-1) = 2 * U(2:end-1);
Y_motor = fft(simout(1:end-1,2)/L);
Y_motor = Y_motor(1:L/2+1);
Y_motor(2:end-1) = 2 * Y_motor(2:end-1);
Y_pen = fft(simout(1:end-1,3)/L);
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

%%
figure(1)
bode(G_1,{f_0*2*pi,f_end*2*pi})
figure(2)
bode(G_2,{f_0*2*pi, f_end*2*pi})


%% Try to cut down the frequency and then fit for that
% G_1 = frd(P1(1:floor(L/2)),f(1:floor(L/2)));
% wt = makeweight(2,1.1,.01,1/Fs);
fit_1 = fitfrd(G_1,10);
% fit_half = fitmagfrd(G_1,10,2,wt);
figure(3);
bode(G_1,fit_1,{f_0*2*pi,f_end*2*pi})
fit_2 = fitfrd(G_2,10);
% fit_half = fitmagfrd(G_1,10,2,wt);
figure(4);
bode(G_2,fit_2,{f_0*2*pi,f_end*2*pi})