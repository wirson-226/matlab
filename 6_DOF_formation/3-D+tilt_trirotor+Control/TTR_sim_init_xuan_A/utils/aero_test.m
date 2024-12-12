% Example inputs
ground_speed = [10, 0, -1, 0, 0, 0, 10, 0, -1,10, 0, -1,10];  % m/s (ground speed)
delta = struct('aileron_l', 0.1, 'aileron_r', -0.1, 'throttle_a', 0.8, 'throttle_b', 0.8,'throttle_c', 0.8, 'arm_a', 0.2, 'arm_b', 0.2);

% Get aircraft parameters from sys_params
params = sys_params();

% Call the function to calculate aerodynamic forces and moments
[force, moment] = all_forces_moments(ground_speed, delta, params);

% Display the results
disp('Forces:');
disp(force);

disp('Moments:');
disp(moment);
