%%
clear;
close all;
clc;
load('raw_data_50.mat')

% The chirp signal parameters(HZ)
f_0 = 0.1;
T = 100;
f_end = 50;
gain = 1;  

%%
qc_build_model('sysID_Simulink');
qc_connect_model('sysID_Simulink');
qc_start_model('sysID_Simulink');

% pause(52);
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

%%
figure(1)
% bode(G_1,{f_0*2*pi,f_end*2*pi})
bode(G_1,{0.1*2*pi,50*2*pi})
figure(2)
% bode(G_2,{f_0*2*pi, f_end*2*pi})
bode(G_2,{0.1*2*pi,50*2*pi})

%% Try to cut down the frequency and then fit for that
% G_1 = frd(P1(1:floor(L/2)),f(1:floor(L/2)));
% wt = makeweight(2,1.1,.01,1/Fs);
fit_1 = fitfrd(G_1,8);
fit_3 = fitfrd(G_3,8);
fit_5 = fitfrd(G_5,8)
% fit_half = fitmagfrd(G_1,10,2,wt);
figure(3);
bodemag(fit_1,fit_3,fit_5,{f_0*2*pi,f_end*2*pi});
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
legend('\fontsize{20}{Fitted Model(1)}','\fontsize{20}{Fitted Model(2)}','\fontsize{20}{Fitted Model(3)}');
grid on;
% xlabel('\fontsize{20}{
fit_2 = fitfrd(G_2,8);
fit_4 = fitfrd(G_4,8);
fit_6 = fitfrd(G_6,8);

% fit_half = fitmagfrd(G_1,10,2,wt);
figure(4);
bodemag(fit_2,fit_4,fit_6,{f_0*2*pi,f_end*2*pi})
set(findall(gcf,'type','line'),'linewidth',3)
set(gca,'FontSize',14)
set(gca,'LineWidth',3)
set(gca,'XColor','k')
set(gca,'YColor','k')
title('Bode Plot for Pendulum')
title('\fontsize{30}{Bode Plot for Pendulum}')
xlabel('\fontsize{20}{Frequency (rad/s)}')
ylabel('\fontsize{20}{Magnitude (dB)}')
legend('\fontsize{20}{Fitted Model(1)}','\fontsize{20}{Fitted Model(2)}','\fontsize{20}{Fitted Model(3)}');
grid on;