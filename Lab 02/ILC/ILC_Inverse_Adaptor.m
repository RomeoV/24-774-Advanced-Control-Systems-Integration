clear;
lqr_setup;
close all;
%% Setup variables
T = 0.002;
T_final = 12;
alpha = .25;
e = [];

timesteps = [0:T:T_final];
u_ilc = [timesteps' zeros(size(timesteps))'];

%% Run a simulation
for i = 1:20
qc_build_model('ILC_InverseController');
qc_connect_model('ILC_InverseController');
qc_start_model('ILC_InverseController');

pause(14);
%% Tune u_ilc
starting_index = floor(2.0/T);
alpha = 0.95*alpha;
% CONSIDER MAKING A PLOT FOR EACH ITERATION PLOTTING THE ERROR
e = [e;std(reference(starting_index:end)-response(starting_index:end))];  %Record error value
for ii = starting_index:size(u_ilc,1)-1
    u_ilc(ii,2) = u_ilc(ii,2)+alpha*(reference(ii+1)-response(ii+1));  %Implement ILC
end
plot(e);
drawnow();
pause(.1);
end