%%
clc;clear;close all;
% The chirp signal parameters(HZ)
f_0 = 0.1;
T = 300;
f_end = 2;
gain = 0.5;

qc_build_model('sysID_Simulink');
qc_connect_model('sysID_Simulink');
qc_start_model('sysID_Simulink');

pause(302);
%%
ydata_1 = simout(:,2);
ydata_2 = simout(:,3);
udata = simout(:,1);

n = 10; %Number of states (order of the fitted system)
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
    Y_2 = [Y_2;ydata_1(ii)];
end
theta_1 = pinv(X_1)*Y_1
theta_2 = pinv(X_2)*Y_2
%%
G_1 = tf(theta_1(11:end)',[1 theta_1(1:10)'],0.02);
G_2 = tf(theta_2(11:end)',[1 theta_2(1:10)'],0.02);
figure(1);
bode(G_1,G_2);
G_1_CT = d2c(G_1);
G_1_CT = d2c(G_2);