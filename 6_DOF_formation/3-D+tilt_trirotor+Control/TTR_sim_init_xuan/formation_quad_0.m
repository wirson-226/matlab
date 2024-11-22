close all;
clear;

addpath('utils');

%%%%% main loop %%%%%

%% pre-calculated trajectories
trajhandle = @traj_line; % 可以运行的轨迹，与时间设定形式有关，配合simulation_3D ----- [F, M, att_des_save_A] = controlhandle(0, current_state_A, desired_state_A, params); 

%% controller selection
controlhandle = @controller_pid;

% Run simulation with given trajectory generator and controller
% state - n x 13, with each row having format [x, y, z, xdot, ydot, zdot, qw, qx, qy, qz, p, q, r]
num_agents = 5; % Number of agents
[t, state] = simulation_3d_att_plus(trajhandle, controlhandle, num_agents); % 实测试版

%%%%%% related functions  %%%%%%

function [desired_state] = traj_line(t, agent_index)
t_max = 12;
t = max(0, min(t, t_max));
t = t / t_max;

pos_base = 10 * t.^3 - 15 * t.^4 + 6 * t.^5;
vel_base = (30 / t_max) * t.^2 - (60 / t_max) * t.^3 + (30 / t_max) * t.^4;
acc_base = (60 / t_max^2) * t - (180 / t_max^2) * t.^2 + (120 / t_max^2) * t.^3;

% Formation offset (regular pentagon in the xy plane)
radius = 1; % Distance from the center of the formation
angle = 2 * pi * (agent_index - 1) / 5; % Angle for pentagon

formation_offset = radius * [cos(angle); sin(angle); 0];

% output desired state
desired_state.pos = formation_offset + [pos_base; 0; 0]; % move along x-axis
desired_state.vel = [vel_base; 0; 0];
desired_state.acc = [acc_base; 0; 0];
desired_state.yaw = 0;
desired_state.yawdot = 0;
end

function [F, M, ATT_des] = controller_pid(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%   ATT_des = [phi_des; theta_des; psi_des];
%   params: robot parameters

% Controller gains
Kp_pos = [5; 5; 5];  % Position proportional gain
Kd_pos = [3; 3; 3];  % Position derivative gain

Kp_att = [20; 20; 20]; % Attitude proportional gain
Kd_att = [5; 5; 5];    % Attitude derivative gain

% Position control
pos_err = des_state.pos - state.pos;
vel_err = des_state.vel - state.vel;

acc_des = des_state.acc + Kp_pos .* pos_err + Kd_pos .* vel_err;

% Desired accelerations
r1_des_ddot = acc_des(1);
r2_des_ddot = acc_des(2);
r3_des_ddot = acc_des(3);

% Desired thrust
u1 = params.mass * (params.gravity + r3_des_ddot);

% Desired roll and pitch angles
phi_des = (r1_des_ddot * sin(des_state.yaw) - r2_des_ddot * cos(des_state.yaw)) / params.gravity;
theta_des = (r1_des_ddot * cos(des_state.yaw) + r2_des_ddot * sin(des_state.yaw)) / params.gravity;
psi_des = des_state.yaw;

% Desired angular velocities (assumed to be zero for hover)
p_des = 0;
q_des = 0;
r_des = des_state.yawdot;

% Current state
phi = state.rot(1);
theta = state.rot(2);
psi = state.rot(3);
p = state.omega(1);
q = state.omega(2);
r = state.omega(3);

% Attitude control using PD control laws
u2 = [Kp_att(1) * (phi_des - phi) + Kd_att(1) * (p_des - p);
      Kp_att(2) * (theta_des - theta) + Kd_att(2) * (q_des - q);
      Kp_att(3) * (psi_des - psi) + Kd_att(3) * (r_des - r)];

% Output force and moments
F = u1;
M = u2;
ATT_des = [phi_des; theta_des; psi_des]; % [3 * 1]
end

function [t_out, s_out] = simulation_3d_att_plus(trajhandle, controlhandle, num_agents)
% NOTE: This script will not run as expected unless you fill in proper
% code in trajhandle and controlhandle
% You should not modify any part of this script except for the
% visualization part
%
% ***************** QUADROTOR SIMULATION *****************

% *********** YOU SHOULDN'T NEED TO CHANGE ANYTHING BELOW **********

addpath('utils');

% real-time
real_time = true;

% max time
max_time = 12;

% parameters for simulation
params = sys_params;

%% **************************** FIGURES *****************************
disp('Initializing figures...');
h_fig = figure;
h_3d = gca;
axis equal
grid on
view(3);
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]')
quadcolors = lines(num_agents);

set(gcf,'Renderer','OpenGL')

%% *********************** INITIAL CONDITIONS ***********************
disp('Setting initial conditions...');
tstep    = 0.01; % this determines the time step at which the solution is given
cstep    = 0.05; % image capture time interval
max_iter = max_time / cstep; % max iteration
nstep    = round(cstep / tstep);
time     = 0; % current time
err = []; % runtime errors

xtraj = zeros(max_iter * nstep, 13, num_agents);
ttraj = zeros(max_iter * nstep, 1);

attitudetraj = zeros(max_iter * nstep, 3, num_agents); % To record attitude (phi, theta, psi)
attitude_des_traj = zeros(max_iter * nstep, 3, num_agents);

for agent = 1:num_agents
    des_start = trajhandle(0, agent);
    random_offset = rand(3, 1) * 2 - 1; % Generate random offset in the range [-1, 1] for x, y, and z
    x0 = init_state(des_start.pos + random_offset, 0); % Add random offset to initial position
    x{agent} = x0; % state for each agent
end

pos_tol = 0.01;
vel_tol = 0.01;

%% ************************* RUN SIMULATION *************************
disp('Simulation Running....');
% Main loop
for iter = 1:max_iter
    timeint = time:tstep:time + cstep;
    tic;

    % Initialize quad plot
    if iter == 1
        for agent = 1:num_agents
            QP(agent) = QuadPlot(agent, x{agent}, 0.1, 0.04, quadcolors(agent, :), max_iter, h_3d);
        end
        h_title = title(sprintf('iteration: %d, time: %4.2f', iter, time));
    end

    % Run simulation
    for agent = 1:num_agents
        [tsave, xsave] = ode45(@(t, s) quadEOM(t, s, controlhandle, @(t, s) trajhandle(t, agent), params), timeint, x{agent});
        x{agent} = xsave(end, :)';

        % Add attitude save
        att_current_save = zeros(length(tsave), 3);
        att_des_save = zeros(length(tsave), 3);

        for i = 1:length(tsave)
            % Current attitude
            current_all_state = stateToQd(xsave(i, :));
            att_current_save(i, :) = current_all_state.rot'; % qd.rot = [phi; theta; yaw]; current [3 * 1]' = [1 * 3]

            % Desired attitude
            desired_state = trajhandle(tsave(i), agent);
            [~, ~, desired_attitude] = controlhandle(tsave(i), current_all_state, desired_state, params);
            att_des_save(i, :) = desired_attitude';
        end

        % Save to traj 每个五步一存记录
        xtraj((iter - 1) * nstep + 1:iter * nstep, :, agent) = xsave(1:end-1, :); % end -1 取消每一步最后状态作为下一步初始的重复
        ttraj((iter - 1) * nstep + 1:iter * nstep) = tsave(1:end-1);

        % Save attitudetraj
        attitude_des_traj((iter - 1) * nstep + 1:iter * nstep, :, agent) = att_des_save(1:end-1, :); % desired att
        attitudetraj((iter - 1) * nstep + 1:iter * nstep, :, agent) = att_current_save(1:end-1, :); % real state att

        % Update quad plot
        current_state = stateToQd(x{agent});
        desired_state = trajhandle(time + cstep, agent);
        QP(agent).UpdateQuadPlot(x{agent}, [desired_state.pos; desired_state.vel], time + cstep);
    end

    set(h_title, 'String', sprintf('iteration: %d, time: %4.2f', iter, time + cstep))

    time = time + cstep; % Update simulation time
    t = toc;

    % Check to make sure ode45 is not timing out
    if (t > cstep * 50)
        err = 'Ode45 Unstable';
        break;
    end

    % Pause to make real-time
    if real_time && (t < cstep)
        pause(cstep - t);
    end

    % Check termination criteria
    if terminate_check(x{1}, time, trajhandle(inf, 1).pos, pos_tol, vel_tol, max_time)
        break;
    end
end

%% ************************* POST PROCESSING *************************

% Truncate xtraj and ttraj  停留在当前步数状态数据，留存以防跳出流失数据
xtraj = xtraj(1:iter * nstep, :, :);
ttraj = ttraj(1:iter * nstep);
attitudetraj = attitudetraj(1:iter * nstep, :, :); % Truncate attitude trajectory
attitude_des_traj = attitude_des_traj(1:iter * nstep, :, :); % Truncate desired attitude trajectory

% Truncate saved variables
for agent = 1:num_agents
    QP(agent).TruncateHist();
end

% Plot position
h_pos = figure('Name', 'Quad Position');
for agent = 1:num_agents
    subplot(3, 1, 1);
    hold on;
    plot(ttraj, xtraj(:, 1, agent), 'LineWidth', 1.5, 'DisplayName', sprintf('Actual x %d', agent));
    plot(ttraj, xtraj(:, 4, agent), '--', 'LineWidth', 1.5, 'DisplayName', sprintf('Desired x %d', agent));
    xlabel('Time [s]');
    ylabel('X Position [m]');
    title('Quad Position - X');
    legend();
    grid on;

    subplot(3, 1, 2);
    hold on;
    plot(ttraj, xtraj(:, 2, agent), 'LineWidth', 1.5, 'DisplayName', sprintf('Actual y %d', agent));
    plot(ttraj, xtraj(:, 5, agent), '--', 'LineWidth', 1.5, 'DisplayName', sprintf('Desired y %d', agent));
    xlabel('Time [s]');
    ylabel('Y Position [m]');
    title('Quad Position - Y');
    legend();
    grid on;

    subplot(3, 1, 3);
    hold on;
    plot(ttraj, xtraj(:, 3, agent), 'LineWidth', 1.5, 'DisplayName', sprintf('Actual z %d', agent));
    plot(ttraj, xtraj(:, 6, agent), '--', 'LineWidth', 1.5, 'DisplayName', sprintf('Desired z %d', agent));
    xlabel('Time [s]');
    ylabel('Z Position [m]');
    title('Quad Position - Z');
    legend();
    grid on;
end

% Plot velocity
h_vel = figure('Name', 'Quad Velocity');
for agent = 1:num_agents
    subplot(3, 1, 1);
    hold on;
    plot(ttraj, xtraj(:, 4, agent), 'LineWidth', 1.5, 'DisplayName', sprintf('Actual xdot %d', agent));
    plot(ttraj, xtraj(:, 7, agent), '--', 'LineWidth', 1.5, 'DisplayName', sprintf('Desired xdot %d', agent));
    xlabel('Time [s]');
    ylabel('X Velocity [m/s]');
    title('Quad Velocity - X');
    legend();
    grid on;

    subplot(3, 1, 2);
    hold on;
    plot(ttraj, xtraj(:, 5, agent), 'LineWidth', 1.5, 'DisplayName', sprintf('Actual ydot %d', agent));
    plot(ttraj, xtraj(:, 8, agent), '--', 'LineWidth', 1.5, 'DisplayName', sprintf('Desired ydot %d', agent));
    xlabel('Time [s]');
    ylabel('Y Velocity [m/s]');
    title('Quad Velocity - Y');
    legend();
    grid on;

    subplot(3, 1, 3);
    hold on;
    plot(ttraj, xtraj(:, 6, agent), 'LineWidth', 1.5, 'DisplayName', sprintf('Actual zdot %d', agent));
    plot(ttraj, xtraj(:, 9, agent), '--', 'LineWidth', 1.5, 'DisplayName', sprintf('Desired zdot %d', agent));
    xlabel('Time [s]');
    ylabel('Z Velocity [m/s]');
    title('Quad Velocity - Z');
    legend();
    grid on;
end

% Plot attitudes

% Plot attitudes
h_attitude = figure('Name', 'Quad Attitude');
colors = lines(num_agents); % Use lines colormap to generate distinct colors

for agent = 1:num_agents
    subplot(3, 1, 1);
    hold on;
    plot(ttraj, rad2deg(attitudetraj(:, 1, agent)), 'Color', colors(agent, :), 'LineWidth', 1.5, 'DisplayName', sprintf('Actual Roll %d', agent));
    plot(ttraj, rad2deg(attitude_des_traj(:, 1, agent)), '--', 'Color', colors(agent, :), 'LineWidth', 1.5, 'DisplayName', sprintf('Desired Roll %d', agent));
    xlabel('Time [s]');
    ylabel('Roll [deg]');
    title('Quad Attitude - Roll');
    legend();
    grid on;

    subplot(3, 1, 2);
    hold on;
    plot(ttraj, rad2deg(attitudetraj(:, 2, agent)), 'Color', colors(agent, :), 'LineWidth', 1.5, 'DisplayName', sprintf('Actual Pitch %d', agent));
    plot(ttraj, rad2deg(attitude_des_traj(:, 2, agent)), '--', 'Color', colors(agent, :), 'LineWidth', 1.5, 'DisplayName', sprintf('Desired Pitch %d', agent));
    xlabel('Time [s]');
    ylabel('Pitch [deg]');
    title('Quad Attitude - Pitch');
    legend();
    grid on;

    subplot(3, 1, 3);
    hold on;
    plot(ttraj, rad2deg(attitudetraj(:, 3, agent)), 'Color', colors(agent, :), 'LineWidth', 1.5, 'DisplayName', sprintf('Actual Yaw %d', agent));
    plot(ttraj, rad2deg(attitude_des_traj(:, 3, agent)), '--', 'Color', colors(agent, :), 'LineWidth', 1.5, 'DisplayName', sprintf('Desired Yaw %d', agent));
    xlabel('Time [s]');
    ylabel('Yaw [deg]');
    title('Quad Attitude - Yaw');
    legend();
    grid on;
end



% h_attitude = figure('Name', 'Quad Attitude');
% for agent = 1:num_agents
%     subplot(3, 1, 1);
%     hold on;
%     plot(ttraj, rad2deg(attitudetraj(:, 1, agent)), 'b', 'LineWidth', 1.5);
%     plot(ttraj, rad2deg(attitude_des_traj(:, 1, agent)), 'r--', 'LineWidth', 1.5);
%     xlabel('Time [s]');
%     ylabel('Roll [deg]');
%     legend('Actual Roll', 'Desired Roll');
%     title('Quad Attitude');
%     grid on;
% 
%     subplot(3, 1, 2);
%     hold on;
%     plot(ttraj, rad2deg(attitudetraj(:, 2, agent)), 'b', 'LineWidth', 1.5);
%     plot(ttraj, rad2deg(attitude_des_traj(:, 2, agent)), 'r--', 'LineWidth', 1.5);
%     xlabel('Time [s]');
%     ylabel('Pitch [deg]');
%     legend('Actual Pitch', 'Desired Pitch');
%     grid on;
% 
%     subplot(3, 1, 3);
%     hold on;
%     plot(ttraj, rad2deg(attitudetraj(:, 3, agent)), 'b', 'LineWidth', 1.5);
%     plot(ttraj, rad2deg(attitude_des_traj(:, 3, agent)), 'r--', 'LineWidth', 1.5);
%     xlabel('Time [s]');
%     ylabel('Yaw [deg]');
%     legend('Actual Yaw', 'Desired Yaw');
%     grid on;
% end

if (~isempty(err))
    error(err);
end

disp('finished.')

t_out = ttraj;
s_out = xtraj;
end
