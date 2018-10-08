clear;
close all;
%% Setup variables
K = 1;
T = 0.003;
T_final = 10;
alpha = .25;
e = [];

timesteps = [0:T:T_final];
u_ilc = [timesteps' zeros(size(timesteps))'];

for i = 1:30
%% Run a simulation
qc_build_model('ILC_Simulink');
qc_connect_model('ILC_Simulink');
qc_start_model('ILC_Simulink');

pause(12);
%% Tune u_ilc
alpha = 0.9*alpha;
e = [e;std(reference-response)];  %Record error value
for ii = 1:size(u_ilc,1)-1
    u_ilc(ii,2) = u_ilc(ii,2)+alpha*(reference(ii+1)-response(ii+1));  %Implement ILC
end
plot(e);
drawnow();
pause(.1);
end