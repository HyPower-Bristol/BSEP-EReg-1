% debug_vectors.m
% Script to debug EReg_Tank_Drain_Init.m vectors

% Run Initialization
try
    EReg_Tank_Drain_Init;
    disp('Initialization successful.');
catch ME
    disp(['Error in Init: ', ME.message]);
    return;
end

% Check sizes
disp(['Time Steps: ', num2str(Num_Steps)]);
disp(['Run Valve Setpoint Length: ', num2str(length(run_valve_setpoint))]);
disp(['Servo Off Setpoint Length: ', num2str(length(servo_off_setpoint))]);

% Check values at 6s (Index 6000 approx)
idx_check = 6000;
if length(run_valve_setpoint) >= idx_check
    val_run = run_valve_setpoint(idx_check);
    val_servo = servo_off_setpoint(idx_check);
    val_time = time(idx_check);

    disp(['Time at index ', num2str(idx_check), ': ', num2str(val_time), ' s']);
    disp(['Run Valve Setpoint at 6s: ', num2str(val_run)]);
    disp(['Servo Off Setpoint at 6s: ', num2str(val_servo)]);

    if val_run == 0
        warning('Run Valve is CLOSED (0) at 6s!');
    else
        disp('Run Valve is OPEN (1) at 6s.');
    end
else
    disp('Simulation Duration < 6s, cannot check index 6000.');
end

% Plot Vectors
figure('Name', 'Debug Vectors');
subplot(3,1,1);
plot(time, setpoint, 'k');
ylabel('Target Pressure (bar)');
title('Setpoint Vectors');
grid on;

subplot(3,1,2);
plot(time, servo_off_setpoint, 'b');
ylabel('Servo Active (1/0)');
grid on;

subplot(3,1,3);
plot(time, run_valve_setpoint, 'r');
ylabel('Run Valve (1=Open)');
xlabel('Time (s)');
grid on;

saveas(gcf, 'debug_vectors.png');
disp('Saved debug_vectors.png');
