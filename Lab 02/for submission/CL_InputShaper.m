
% Collect transient responnse data from giving system an impulse
clear;
close all;
clc;
fprintf("################### Get Transient Response ###################");
% Run the simulink model to gather impulse response
model_name = 'CL_Transient_Response';
Ts = 0.002; % Sampling time
T = 10;
qc_build_model(model_name);
qc_connect_model(model_name);
qc_start_model(model_name);

% wait for the system to gather data
pause(T+10);
%% 

% The pendulum response is saved
n = size(response,1);
t_trans = [1:n]*T/n;
plot(t_trans,response,'LineWidth',2);
xlabel('second');
ylabel('rad');
title('Transient Response');
set(gca,'fontsize',20);


[peaks, locations] = findpeaks(response);
% change time according to sampling time
time = locations * Ts;
% get the damping ration and natural frequency by calculating the ratio
t1 = time(1);
y1 = peaks(1);
t2 = time(2);
y2 = peaks(2);


Td_nom = t2 - t1; % cycle of the damped system
wd = 2*pi/Td_nom; % wd = wn * sqrt(1 - zetta^2)
zeta_wn = (-log(y2/y1)) / (t2 - t1);
A = (zeta_wn/wd)^2;
zeta_nom = sqrt(A/(1+A));




%% 

% Use the natural frequency and damping ratio to build input shaper
Td = Td_nom * 0.9;
zeta = zeta_nom * 0.9;

K = exp(-zeta*pi/sqrt(1-zeta^2));
A0 = 1/(1+2*K+K^2);
A1 = 2*K/(1+2*K+K^2);
A2 = K^2/(1+2*K+K^2);
t1 = Td/2;
t2 = Td;
%% 

% Run the simulink model to compare shaped and not shaped results
model_name = 'CL_Input_Shaping';
T = 10;
% First gather data of orignal reference
shape_ref = 0;
qc_build_model(model_name);
qc_connect_model(model_name);
qc_start_model(model_name);
pause(T + 10);

pendulum_unshaped = tracking_response(:,1);
motor_unshaped = tracking_response(:,2);
motor_ref = tracking_response(:,3);
%% 


% then gather data of shaped reference
shape_ref = 1;
qc_build_model(model_name);
qc_connect_model(model_name);
qc_start_model(model_name);
pause(T+10);

pendulum_shaped = tracking_response(:,1);
motor_shaped = tracking_response(:,2);


%% 
% plot the response comparison
figure
n = size(pendulum_unshaped,1);
t = [1:n] * T / n;
plot(t,pendulum_unshaped,'LineWidth',2);
hold on
plot(t,pendulum_shaped,'LineWidth',2);
legend("unshaped", "shaped");
title('pendulum');
xlabel('second');
ylabel('rad');
set(gca,'fontsize',20)

figure;
plot(t,motor_ref,'LineWidth',2);
hold on
plot(t,motor_unshaped,'LineWidth',2);
plot(t,motor_shaped,'k','LineWidth',2);
legend("reference","unshaped", "shaped");
title("motor");
xlabel('second');
ylabel('rad');
set(gca,'fontsize',20);




