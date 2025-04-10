function [t_out, s_out] = simulation_3d_basic(trajhandle, controlhandle, num_agents)
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

global_state.pos = zeros(3, num_agents);
global_state.vel = zeros(3, num_agents);
global_state.rot = zeros(3, num_agents);
global_state.omega = zeros(3, num_agents);

% % Variables to hold plot handles for real-time polygon visualization
% real_time_polygon = plot3(h_3d, NaN, NaN, NaN, 'r-', 'LineWidth', 1.5);
% desired_polygon = plot3(h_3d, NaN, NaN, NaN, 'g--', 'LineWidth', 1.5);

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

    % Generate global desired state
    global_des_state = generate_global_des_state(trajhandle, time, num_agents);

    % Run simulation

    for agent = 1:num_agents
        [tsave, xsave] = ode45(@(t, s) quadEOM(t, s, controlhandle, trajhandle, params, global_state, global_des_state, agent), timeint, x{agent});
        x{agent} = xsave(end, :)';

        % Update global state
        qd = stateToQd(x{agent});
        global_state.pos(:, agent) = qd.pos;
        global_state.vel(:, agent) = qd.vel;
        global_state.rot(:, agent) = qd.rot;
        global_state.omega(:, agent) = qd.omega;

        % Add attitude save
        att_current_save = zeros(length(tsave), 3);
        att_des_save = zeros(length(tsave), 3);

        for i = 1:length(tsave)
            % Current attitude
            current_all_state = stateToQd(xsave(i, :));
            att_current_save(i, :) = current_all_state.rot'; % qd.rot = [phi; theta; yaw]; current [3 * 1]' = [1 * 3]

            % Desired attitude
            desired_state = trajhandle(tsave(i), agent);
            [~, ~, desired_attitude] = controlhandle(tsave(i), current_all_state, desired_state, params, global_state, global_des_state, agent);
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

        
        % Plot agent connections (polygon)
    if iter > 1 && mod(iter, 5) == 0 % Plot every 5 iterations
        % Clear previous plots
        if exist('polygon_handle', 'var') && ishandle(polygon_handle)
            delete(polygon_handle);
        end
        if exist('polygon_handle_des', 'var') && ishandle(polygon_handle_des)
            delete(polygon_handle_des);
        end
        % if exist('markers_des', 'var') && ishandle(markers_des)
        %     delete(markers_des);
        % end

        % Check if markers_des variable exists and is a valid handle
        if exist('markers_des', 'var')
            for agent = 1:num_agents
                if ishandle(markers_des(agent))
                    delete(markers_des(agent));
                end
            end
        end
    
        % Extract current positions of all agents and desired positions
        current_positions = zeros(3, num_agents);
        desired_positions = zeros(3, num_agents);
        for agent = 1:num_agents
            current_positions(:, agent) = x{agent}(1:3); % Extract current position of each agent
            des_state = trajhandle(time + cstep, agent);
            desired_positions(:, agent) = des_state.pos; % Extract desired position from trajhandle
        end
    
        % Plot polygon (connections) for current positions
        hold on;
        polygon_vertices = [current_positions, current_positions(:, 1)]; % Close polygon
        polygon_handle = plot3(h_3d, polygon_vertices(1, :), polygon_vertices(2, :), polygon_vertices(3, :), 'LineStyle', '--', 'Color', 'k', 'LineWidth', 1.5);
    
        % Plot polygon (connections) for desired positions and markers
        polygon_vertices_des = [desired_positions, desired_positions(:, 1)]; % Close polygon
        polygon_handle_des = plot3(h_3d, polygon_vertices_des(1, :), polygon_vertices_des(2, :), polygon_vertices_des(3, :), 'LineStyle', '--', 'Color', 'g', 'LineWidth', 1.5);
    
        % % Plot shape markers at desired positions
        % markers_des = plot3(h_3d, desired_positions(1, :), desired_positions(2, :), desired_positions(3, :), 'o', 'Color', quadcolors(agent, :), 'MarkerFaceColor', quadcolors(agent, :), 'MarkerSize', 8);
        % 
        % Plot shape markers at desired positions with agent colors
        markers_des = gobjects(num_agents, 1); % Initialize array to store markers
        for agent = 1:num_agents
            agent_color = quadcolors(agent, :); % Get the color for the current agent
            markers_des(agent) = plot3(h_3d, desired_positions(1, agent), desired_positions(2, agent), desired_positions(3, agent), ...
                'o', 'Color', agent_color, 'MarkerFaceColor', agent_color, 'MarkerSize', 8);
        end
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

if (~isempty(err))
    error(err);
end

disp('finished.')

t_out = ttraj;
s_out = xtraj;
end
