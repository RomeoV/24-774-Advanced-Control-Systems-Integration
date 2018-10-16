%%
clc;clear;close all;
load('raw_data_30.mat')
% The chirp signal parameters(HZ)
% f_0 = 0.1;
T = 30;
% f_end = 100;
gain = 1;
%%
qc_build_model('sysID_param');
qc_connect_model('sysID_param');
qc_start_model('sysID_param');

pause(32);
%%
% load raw_data_30
ydata_1 = simout(:,2);%motor
ydata_2 = simout(:,3);%pendulum
udata = simout(:,1);

n = 5; %Number of states (order of the fitted system)
L = length(udata);
X_1 = [];
X_2 = [];
Y_1 = [];
Y_2 = [];
for ii = n+1:L
    y_1 = [];
    y_2 = [];
    u = [];
    for jj = 1:n
        y_1 = [y_1 -ydata_1(ii-jj)];
        y_2 = [y_2 -ydata_2(ii-jj)];
        u = [u udata(ii-jj)];
    end
    X_1 = [X_1;[y_1,u]];
    Y_1 = [Y_1;ydata_1(ii)];
    X_2 = [X_2;[y_2,u]];
    Y_2 = [Y_2;ydata_2(ii)];
end
theta_1 = pinv(X_1)*Y_1; %motor
theta_2 = pinv(X_2)*Y_2; %pendulum
%%
load('G_motor_real.mat')
load('G_pen_real.mat')
G1 = tf(theta_1(n+1:end)',[1 theta_1(1:n)'],0.01);
G2 = tf(theta_2(n+1:end)',[1 theta_2(1:n)'],0.01);
% figure(1);
% bode(G1,G2);
G_1_CT = d2c(G1);
G_2_CT = d2c(G2);
% G_CT = minreal([tf(G_1_CT);tf(G_2_CT)]);
figure(1)
bodemag(G_1,G_1_CT,{0.1*2*pi,50*2*pi});
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
legend('\fontsize{20}{Measured Model}','\fontsize{20}{Fitted Model}');
grid on;
figure(2)
bodemag(G_2,G_2_CT,{0.1*2*pi,50*2*pi});
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
legend('\fontsize{20}{Measured Model}','\fontsize{20}{Fitted Model}');
grid on;