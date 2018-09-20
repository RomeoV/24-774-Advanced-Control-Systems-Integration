tic();
T_pitch = 2*pi*1/.4;
T_yaw = 2*pi*1/.5;
x = linspace(0,50,400);
y_pitch = zeros(1,400);
y_yaw = zeros(1,400);
for i = 1:400
    phase_pitch = mod(toc(),T_pitch);
    phase_yaw = mod(toc(),T_yaw);
    if phase_pitch <= T_pitch/2
        y_pitch(i) = -pi/6;
    elseif phase_pitch > T_pitch/2
        y_pitch(i) = pi/6;
    end
    
    if phase_yaw <= T_yaw/2
        y_yaw(i) = -pi/4;
    elseif phase_yaw > T_yaw/2
        y_yaw(i) = pi/4;
    end
    plot(x,y_pitch,x,y_yaw,'LineWidth',8);
    ylim([-1 1])
    drawnow();
    pause(.1);
end