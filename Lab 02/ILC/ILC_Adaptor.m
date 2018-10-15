clear;
close all;
%% Setup variables
T = 0.003;
T_final = 10;
alpha = .99;
e = [];
Kp = .5;
Kd = 0.1; 
timesteps = [0:T:T_final];
u_ilc = [timesteps' zeros(size(timesteps))'];

%% Run a simulation
for i = 1:20
qc_build_model('ILC_Simulink');
qc_connect_model('ILC_Simulink');
qc_start_model('ILC_Simulink');

pause(12);
%% Tune u_ilc
alpha = 0.95*alpha;
e = [e;std(reference-response)];  %Record error value
for ii = 1:size(u_ilc,1)-1
    u_ilc(ii,2) = u_ilc(ii,2)+alpha*(reference(ii+1)-response(ii+1));  %Implement ILC
end
plot(e);
drawnow();
pause(.1);
end