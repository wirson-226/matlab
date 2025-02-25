function [t_out, s_out] = simulation_simple(trajhandle, controlhandle)
% Optimized quadrotor simulation with preallocated memory and fixed-step RK4 integrator.
% Assumes trajhandle and controlhandle are properly implemented.

%% Add paths
addpath('utils');
addpath('traj');
addpath('controller');
addpath('test_tools');
addpath('test_airplane');

%% Simulation parameters
real_time = true;
max_time = 6; % Maximum simulation time
tstep = 0.02; % Time step for fixed-step integrator
cstep = 0.05; % Image capture time interval
max_iter = max_time / cstep; % Maximum iterations
nstep = round(cstep / tstep); % Steps per iteration
time = 0; % Initialize time
params = sys_params; % Load system parameters

%% Preallocate memory
disp('Initializing simulation...');
x0 = init_state(trajhandle(0, []).pos, 0);
state_dim = length(x0); % State dimension
xtraj = zeros(max_iter * nstep, state_dim); % State trajectory
ttraj = zeros(max_iter * nstep, 1); % Time trajectory
actual_trajectory = zeros(max_iter * nstep, 3); % Actual positions
desired_trajectory = zeros(max_iter * nstep, 3); % Desired positions

% Preallocate additional logs
attitudetraj = zeros(max_iter * nstep, 3); % Attitude (phi, theta, psi)
omegatraj = zeros(max_iter * nstep, 3); % Angular velocity (p, q, r)
position_des_traj = zeros(max_iter * nstep, 3); % Desired position
velocity_des_traj = zeros(max_iter * nstep, 3); % Desired velocity
attitude_des_traj = zeros(max_iter * nstep, 3); % Desired attitude
omega_des_traj = zeros(max_iter * nstep, 3); % Desired angular velocity
M_des_traj = zeros(max_iter * nstep, 3); % Desired moments

% Actuator inputs
tilt_des_traj = zeros(max_iter * nstep, 2); % Tilt angles
throttle_des_traj = zeros(max_iter * nstep, 3); % Throttle inputs
elevon_des_traj = zeros(max_iter * nstep, 2); % Elevon angles

%% Initialize state
x = x0; % Initial state
pos_tol = 0.01; % Position tolerance
vel_tol = 0.01; % Velocity tolerance

%% Fixed-step RK4 integrator
disp('Running simulation...');
for iter = 1:max_iter
    timeint = time:tstep:time + cstep; % Time interval for each iteration
    for t_idx = 1:length(timeint)-1
        x = RK4(@vtolEOM, timeint(t_idx), x, tstep, controlhandle, trajhandle, params);
    end
    
    % Save states and compute trajectories
    actual_trajectory((iter-1)*nstep+1:iter*nstep, :) = repmat(x(1:3)', nstep, 1);
    current_all_state = stateToQd(x);
    attitudetraj((iter-1)*nstep+1:iter*nstep, :) = repmat(current_all_state.rot', nstep, 1);
    % omegatraj((iter-1)*nstep+1:iter*nstep, :) = repmat(current_all_state.omega, nstep, 1);

    desired_state = trajhandle(time, current_all_state);
    desired_trajectory((iter-1)*nstep+1:iter*nstep, :) = repmat(desired_state.pos', nstep, 1);
    position_des_traj((iter-1)*nstep+1:iter*nstep, :) = repmat(desired_state.pos', nstep, 1);

    % Save actuator inputs
    [des_from_ctrl, command] = controlhandle(time, current_all_state, desired_state, params);
    % tilt_des_traj((iter-1)*nstep+1:iter*nstep, :) = command.arm;
    % throttle_des_traj((iter-1)*nstep+1:iter*nstep, :) = command.throttle;
    % elevon_des_traj((iter-1)*nstep+1:iter*nstep, :) = command.elevon;

    % Update time
    time = time + cstep;

    % Pause for real-time simulation
    if real_time
        pause(cstep);
    end

    % Check termination criteria
    if terminate_check(x, time, trajhandle(inf, []).pos, pos_tol, vel_tol, max_time)
        break;
    end
end

%% Truncate results to final iteration
xtraj = xtraj(1:iter*nstep, :);
ttraj = ttraj(1:iter*nstep);
actual_trajectory = actual_trajectory(1:iter*nstep, :);
desired_trajectory = desired_trajectory(1:iter*nstep, :);

%% Plot results
disp('Plotting results...');
plot_trajectory(ttraj, actual_trajectory, desired_trajectory);
plot_attitude(ttraj, attitudetraj, attitude_des_traj);

disp('Simulation finished.');
t_out = ttraj;
s_out = xtraj;

end

%% Fixed-step RK4 integrator
function [x_next] = RK4(f, t, x, dt, controlhandle, trajhandle, params)
    k1 = f(t, x, controlhandle, trajhandle, params);
    k2 = f(t + dt/2, x + dt/2 * k1, controlhandle, trajhandle, params);
    k3 = f(t + dt/2, x + dt/2 * k2, controlhandle, trajhandle, params);
    k4 = f(t + dt, x + dt * k3, controlhandle, trajhandle, params);
    x_next = x + dt/6 * (k1 + 2*k2 + 2*k3 + k4);
end

%% Plot trajectory
function plot_trajectory(ttraj, actual_trajectory, desired_trajectory)
    figure('Name', 'Trajectory');
    plot3(actual_trajectory(:, 1), actual_trajectory(:, 2), actual_trajectory(:, 3), 'b-', 'LineWidth', 1.5);
    hold on;
    plot3(desired_trajectory(:, 1), desired_trajectory(:, 2), desired_trajectory(:, 3), 'r--', 'LineWidth', 1.5);
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    legend('Actual Trajectory', 'Desired Trajectory');
    grid on;
    title('3D Trajectory');
end

%% Plot attitude
function plot_attitude(ttraj, attitudetraj, attitude_des_traj)
    figure('Name', 'Attitude');
    subplot(3,1,1);
    plot(ttraj, rad2deg(attitudetraj(:,1)), 'b', 'LineWidth', 1.5);
    hold on;
    plot(ttraj, rad2deg(attitude_des_traj(:,1)), 'r--', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Roll [deg]');
    legend('Actual Roll', 'Desired Roll');
    grid on;

    subplot(3,1,2);
    plot(ttraj, rad2deg(attitudetraj(:,2)), 'b', 'LineWidth', 1.5);
    hold on;
    plot(ttraj, rad2deg(attitude_des_traj(:,2)), 'r--', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Pitch [deg]');
    legend('Actual Pitch', 'Desired Pitch');
    grid on;

    subplot(3,1,3);
    plot(ttraj, rad2deg(attitudetraj(:,3)), 'b', 'LineWidth', 1.5);
    hold on;
    plot(ttraj, rad2deg(attitude_des_traj(:,3)), 'r--', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Yaw [deg]');
    legend('Actual Yaw', 'Desired Yaw');
    grid on;
end
