clear;
qc_build_model('sysID_Simulink');
qc_connect_model('sysID_Simulink');
qc_start_model('sysID_Simulink');

pause(12);
L = length(simout(:,1));
Fs = 1000;

U = fft(simout(:,1));
Y_1 = fft(simout(:,2));
Y_2 = fft(simout(:,3));

fs = Fs*(0:L/2)/L;

G_1 = frd(Y_1/U,fs);
G_2 = frd(Y_2/U,fs);

bode(G_1,G_2)