function [t_out, s_out] = simulation_formation_ttr(trajhandle, controlhandle, num_agents)
% 多智能体 TTR-VTOL 仿真框架，支持多架无人机（如六架）同时运行

addpath('utils');
addpath('traj');
addpath('controller');
addpath('test_airplane');

% ====== SIMULATION CONFIG ======
real_time = evalin('base', 'exist(''real_time'', ''var'') && real_time');
if evalin('base', 'exist(''max_time'', ''var'')')
    max_time = evalin('base', 'max_time');
else
    max_time = 5;  % 默认值
end

% Initialize stop flag if not exists
if ~evalin('base', 'exist(''stop_simulation'', ''var'')')
    assignin('base', 'stop_simulation', false);
end

tstep    = 0.01;        % 状态积分步长
cstep    = 0.05;        % 控制周期（也是绘图周期）
max_iter = max_time / cstep;
nstep    = round(cstep / tstep);

% ====== SYSTEM PARAMS ======
params = sys_params;

% ====== FIGURE SETUP（首次只初始化一次） ======
disp('Initializing figure...');
figure(1); clf;
axis equal; grid on; hold on;
xlabel('X_东_右'); ylabel('Y_北_前'); zlabel('Z_上');
title('Multi-TTR-VTOL Flight');
view(3);
colors = lines(num_agents);

% 保留句柄用于动态更新
h_planes = cell(num_agents, 1);  % 替换原句柄初始化        
h_lines = gobjects(num_agents, 1);      % 连线

% ====== STATE INITIALIZATION ======
disp('Setting initial conditions...');
state_dim = 13;
X = zeros(state_dim, num_agents); % 所有无人机的状态矩阵

for i = 1:num_agents
    des_i = trajhandle(0, i);
    X(:, i) = init_state(des_i.pos, params.psi0);
end

% Initialize trajectory storage
[xtraj, ttraj, actual_trajectory, desired_trajectory] = deal(cell(1, num_agents));
[attitudetraj, omegatraj, attitude_des_traj, omega_des_traj] = deal(cell(1, num_agents));
[position_des_traj, velocity_des_traj, M_des_traj] = deal(cell(1, num_agents));
[tilt_des_traj, throttle_des_traj, elevon_des_traj] = deal(cell(1, num_agents));

% ====== SIMULATION LOOP ======
disp('Simulation Running...');
time = 0;
for iter = 1:max_iter
    % Check stop flag from base workspace
    if evalin('base', 'stop_simulation')
        disp('Simulation stopped by user request.');
        break;
    end
    
    timeint = time:tstep:time+cstep;
    tic;

    formation_positions = zeros(num_agents, 3);

    for i = 1:num_agents
        % Check stop flag again in case it changed during iteration
        if evalin('base', 'stop_simulation')
            break;
        end
        
        x = X(:, i);
        [tsave, xsave] = ode45(@(t,s) vtolEOM(t, s, controlhandle, @(t,~) trajhandle(t,i), params), timeint, x);
        X(:, i) = xsave(end, :)';

        % Store trajectory data
        if iter == 1
            xtraj{i} = xsave(1:end-1, :);
            ttraj{i} = tsave(1:end-1);
        else
            xtraj{i} = [xtraj{i}; xsave(1:end-1,:)];
            ttraj{i} = [ttraj{i}; tsave(1:end-1)];
        end

        % Get current state and desired values
        qd_i = stateToQd(X(:, i));
        des_i = trajhandle(time, i);
        [des_ctrl_i, command_i, ~] = controlhandle(time, qd_i, des_i, params);

        % Update visualization
        R = QuatToRot(X(7:10, i));
        [phi, theta, psi] = RotToRPY_ZXY(R);
        att_i = [phi, theta, psi];
        tilt_deg = rad2deg(command_i.arm);

        % Delete old plane plot if it exists
        if ~isempty(h_planes{i}) && all(isgraphics(h_planes{i}))
            delete(h_planes{i});
        end

        % Plot current state
        h_planes{i} = planeplot_ttr(X(1:3, i)', att_i, tilt_deg);
        
        % Update trajectories
        actual_trajectory{i} = [actual_trajectory{i}; X(1:3, i)'];
        desired_trajectory{i} = [desired_trajectory{i}; des_i.pos'];
        
        % Plot trajectories
        if size(actual_trajectory{i},1) >= 2
            plot3(actual_trajectory{i}(end-1:end,1), actual_trajectory{i}(end-1:end,2),...
                 actual_trajectory{i}(end-1:end,3), '-', 'Color', colors(i,:), 'LineWidth', 1.5);
        else
            plot3(actual_trajectory{i}(end,1), actual_trajectory{i}(end,2),...
                 actual_trajectory{i}(end,3), '.', 'Color', colors(i,:), 'LineWidth', 1.5);
        end
        
        if size(desired_trajectory{i},1) >= 2
            plot3(desired_trajectory{i}(end-1:end,1), desired_trajectory{i}(end-1:end,2),...
                 desired_trajectory{i}(end-1:end,3), '--', 'Color', colors(i,:), 'LineWidth', 1);
        else
            plot3(desired_trajectory{i}(end,1), desired_trajectory{i}(end,2),...
                 desired_trajectory{i}(end,3), 'x', 'Color', colors(i,:), 'LineWidth', 1);
        end

        formation_positions(i, :) = X(1:3, i)';

        % Store additional data
        attitudetraj{i} = [attitudetraj{i}; att_i];
        omegatraj{i} = [omegatraj{i}; qd_i.omega'];
        position_des_traj{i} = [position_des_traj{i}; des_i.pos'];
        velocity_des_traj{i} = [velocity_des_traj{i}; des_ctrl_i(1:3)];
        attitude_des_traj{i} = [attitude_des_traj{i}; des_ctrl_i(4:6)];
        omega_des_traj{i} = [omega_des_traj{i}; des_ctrl_i(7:9)];
        M_des_traj{i} = [M_des_traj{i}; des_ctrl_i(10:12)];

        tilt_des_traj{i} = [tilt_des_traj{i}; command_i.arm];
        throttle_des_traj{i} = [throttle_des_traj{i}; command_i.throttle];
        elevon_des_traj{i} = [elevon_des_traj{i}; command_i.elevon];
    end

    % Check if we broke out of the agent loop due to stop request
    if evalin('base', 'stop_simulation')
        break;
    end

    % ====== 连线更新 ======
    for j = 1:num_agents
        if isgraphics(h_lines(j))
            delete(h_lines(j));
        end
        idx_next = mod(j, num_agents) + 1;
        h_lines(j) = plot3([formation_positions(j,1), formation_positions(idx_next,1)], ...
                         [formation_positions(j,2), formation_positions(idx_next,2)], ...
                         [formation_positions(j,3), formation_positions(idx_next,3)], ...
                         'k--', 'LineWidth', 1);
    end

    % Real-time pacing
    t = toc;
    if real_time && (t < cstep)
        pause(cstep - t);
    end

    time = time + cstep;
    drawnow limitrate;
end

% Reset stop flag for next simulation
assignin('base', 'stop_simulation', false);

% ====== 数据输出与整合 ======
if ~isempty(ttraj{1})
    t_out = ttraj{1};
    x_out = xtraj;  % <== 直接返回 cell 数组
else
    t_out = [];
    x_out = {};
end

desired_out = struct();
actual_out = struct();
actuator_out = struct();

for i = 1:num_agents
    actual_out.att{i}     = attitudetraj{i};
    actual_out.omega{i}   = omegatraj{i};
    desired_out.pos{i}    = position_des_traj{i};
    desired_out.vel{i}    = velocity_des_traj{i};
    desired_out.att{i}    = attitude_des_traj{i};
    desired_out.omega{i}  = omega_des_traj{i};
    desired_out.M{i}      = M_des_traj{i};
    actuator_out.tilt{i}  = tilt_des_traj{i};
    actuator_out.throttle{i} = throttle_des_traj{i};
    actuator_out.elevon{i} = elevon_des_traj{i};
end

% 调用绘图函数
plot_results_multi(t_out, x_out, desired_out, actual_out, actuator_out, num_agents, colors);

% 图像导出
save_dir = 'D:\Codes\Matlab_Xuan\matlab\6_DOF_formation\TTR_formation_sim_init\Medias\Results';
if ~exist(save_dir, 'dir')
    mkdir(save_dir);
end
figHandles = findall(0, 'Type', 'figure');
for i = 1:length(figHandles)
    figure(figHandles(i));
    figName = get(figHandles(i), 'Name');
    if isempty(figName)
        figName = ['Figure' num2str(figHandles(i).Number)];
    end
    saveas(figHandles(i), fullfile(save_dir, [figName '.png']));
end

disp('Simulation complete.');
end
