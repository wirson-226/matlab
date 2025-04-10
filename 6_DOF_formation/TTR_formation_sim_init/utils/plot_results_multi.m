function plot_results_multi(t, x, desired, actual, actuator, num_agents, colors)
% 多无人机统一绘图函数，展示位置、速度、姿态、角速度、执行器与控制矩
% 输入:
%   t - 时间向量
%   x - 包含所有无人机状态的元胞数组
%   desired - 期望轨迹的结构体，包含pos,vel,att,omega,M的元胞数组字段
%   actual - 实际轨迹的结构体，包含att,omega的元胞数组字段
%   actuator - 执行器的结构体，包含tilt,throttle,elevon的元胞数组字段
%   num_agents - 无人机数量
%   colors - 颜色矩阵，每行代表一个无人机的颜色

% 检查输入参数
if isempty(t) || isempty(x)
    disp('No trajectory data to plot.');
    return;
end

% 确保所有向量长度一致
t_length = length(t);
for i = 1:num_agents
    % 检查并确保各个向量长度与时间向量相匹配
    if size(desired.pos{i}, 1) ~= t_length
        desired.pos{i} = interp1(linspace(0, 1, size(desired.pos{i}, 1)), desired.pos{i}, linspace(0, 1, t_length));
    end
    if size(desired.vel{i}, 1) ~= t_length
        desired.vel{i} = interp1(linspace(0, 1, size(desired.vel{i}, 1)), desired.vel{i}, linspace(0, 1, t_length));
    end
    if size(desired.att{i}, 1) ~= t_length
        desired.att{i} = interp1(linspace(0, 1, size(desired.att{i}, 1)), desired.att{i}, linspace(0, 1, t_length));
    end
    if size(desired.omega{i}, 1) ~= t_length
        desired.omega{i} = interp1(linspace(0, 1, size(desired.omega{i}, 1)), desired.omega{i}, linspace(0, 1, t_length));
    end
    if size(desired.M{i}, 1) ~= t_length
        desired.M{i} = interp1(linspace(0, 1, size(desired.M{i}, 1)), desired.M{i}, linspace(0, 1, t_length));
    end
    if size(actual.att{i}, 1) ~= t_length
        actual.att{i} = interp1(linspace(0, 1, size(actual.att{i}, 1)), actual.att{i}, linspace(0, 1, t_length));
    end
    if size(actual.omega{i}, 1) ~= t_length
        actual.omega{i} = interp1(linspace(0, 1, size(actual.omega{i}, 1)), actual.omega{i}, linspace(0, 1, t_length));
    end
    if size(actuator.tilt{i}, 1) ~= t_length
        actuator.tilt{i} = interp1(linspace(0, 1, size(actuator.tilt{i}, 1)), actuator.tilt{i}, linspace(0, 1, t_length));
    end
    if size(actuator.throttle{i}, 1) ~= t_length
        actuator.throttle{i} = interp1(linspace(0, 1, size(actuator.throttle{i}, 1)), actuator.throttle{i}, linspace(0, 1, t_length));
    end
    if size(actuator.elevon{i}, 1) ~= t_length
        actuator.elevon{i} = interp1(linspace(0, 1, size(actuator.elevon{i}, 1)), actuator.elevon{i}, linspace(0, 1, t_length));
    end
    
    % 检查状态向量
    if size(x{i}, 1) ~= t_length
        % 状态向量通常是 x, y, z, vx, vy, vz, qw, qx, qy, qz, ...
        % 由于状态向量包含四元数等复杂数据，这里仅进行简单的线性插值
        x{i} = interp1(linspace(0, 1, size(x{i}, 1)), x{i}, linspace(0, 1, t_length));
    end
end

% 创建新的图形窗口，避免覆盖主仿真窗口
figure_names = {'Position', 'Velocity', 'Attitude', 'Angular Velocity', 'Actuators', 'Control Moments'};
handles = cell(length(figure_names), 1);

for i = 1:length(figure_names)
    handles{i} = figure('Name', figure_names{i}, 'NumberTitle', 'off');
end

% 位置图
figure(handles{1});
subplot(3, 1, 1);
hold on; grid on;
title('X Position');
xlabel('Time (s)');
ylabel('X (m)');
legend_str = cell(1, 2*num_agents);
for i = 1:num_agents
    try
        % 实际位置
        plot(t, x{i}(:, 1), 'Color', colors(i, :), 'LineWidth', 1.5);
        % 期望位置
        plot(t, desired.pos{i}(:, 1), '--', 'Color', colors(i, :), 'LineWidth', 1);
        
        legend_str{2*i-1} = sprintf('Agent %d Actual', i);
        legend_str{2*i} = sprintf('Agent %d Desired', i);
    catch e
        warning('Error plotting position for agent %d: %s', i, e.message);
    end
end
legend(legend_str, 'Location', 'best', 'NumColumns', min(num_agents, 3));

subplot(3, 1, 2);
hold on; grid on;
title('Y Position');
xlabel('Time (s)');
ylabel('Y (m)');
for i = 1:num_agents
    try
        plot(t, x{i}(:, 2), 'Color', colors(i, :), 'LineWidth', 1.5);
        plot(t, desired.pos{i}(:, 2), '--', 'Color', colors(i, :), 'LineWidth', 1);
    catch e
        warning('Error plotting position Y for agent %d: %s', i, e.message);
    end
end

subplot(3, 1, 3);
hold on; grid on;
title('Z Position');
xlabel('Time (s)');
ylabel('Z (m)');
for i = 1:num_agents
    try
        plot(t, x{i}(:, 3), 'Color', colors(i, :), 'LineWidth', 1.5);
        plot(t, desired.pos{i}(:, 3), '--', 'Color', colors(i, :), 'LineWidth', 1);
    catch e
        warning('Error plotting position Z for agent %d: %s', i, e.message);
    end
end

% 计算位置误差
figure('Name', 'Position Error', 'NumberTitle', 'off');
subplot(3, 1, 1);
hold on; grid on;
title('X Position Error');
xlabel('Time (s)');
ylabel('Error (m)');
legend_str_single = cell(1, num_agents);
for i = 1:num_agents
    try
        plot(t, x{i}(:, 1) - desired.pos{i}(:, 1), 'Color', colors(i, :), 'LineWidth', 1.5);
        legend_str_single{i} = sprintf('Agent %d', i);
    catch e
        warning('Error plotting position error X for agent %d: %s', i, e.message);
    end
end
legend(legend_str_single, 'Location', 'best');

subplot(3, 1, 2);
hold on; grid on;
title('Y Position Error');
xlabel('Time (s)');
ylabel('Error (m)');
for i = 1:num_agents
    try
        plot(t, x{i}(:, 2) - desired.pos{i}(:, 2), 'Color', colors(i, :), 'LineWidth', 1.5);
    catch e
        warning('Error plotting position error Y for agent %d: %s', i, e.message);
    end
end

subplot(3, 1, 3);
hold on; grid on;
title('Z Position Error');
xlabel('Time (s)');
ylabel('Error (m)');
for i = 1:num_agents
    try
        plot(t, x{i}(:, 3) - desired.pos{i}(:, 3), 'Color', colors(i, :), 'LineWidth', 1.5);
    catch e
        warning('Error plotting position error Z for agent %d: %s', i, e.message);
    end
end

% 速度图
figure(handles{2});
subplot(3, 1, 1);
hold on; grid on;
title('X Velocity');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
for i = 1:num_agents
    try
        plot(t, x{i}(:, 4), 'Color', colors(i, :), 'LineWidth', 1.5);
        plot(t, desired.vel{i}(:, 1), '--', 'Color', colors(i, :), 'LineWidth', 1);
    catch e
        warning('Error plotting velocity X for agent %d: %s', i, e.message);
    end
end
legend(legend_str, 'Location', 'best', 'NumColumns', min(num_agents, 3));

subplot(3, 1, 2);
hold on; grid on;
title('Y Velocity');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
for i = 1:num_agents
    try
        plot(t, x{i}(:, 5), 'Color', colors(i, :), 'LineWidth', 1.5);
        plot(t, desired.vel{i}(:, 2), '--', 'Color', colors(i, :), 'LineWidth', 1);
    catch e
        warning('Error plotting velocity Y for agent %d: %s', i, e.message);
    end
end

subplot(3, 1, 3);
hold on; grid on;
title('Z Velocity');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
for i = 1:num_agents
    try
        plot(t, x{i}(:, 6), 'Color', colors(i, :), 'LineWidth', 1.5);
        plot(t, desired.vel{i}(:, 3), '--', 'Color', colors(i, :), 'LineWidth', 1);
    catch e
        warning('Error plotting velocity Z for agent %d: %s', i, e.message);
    end
end

% 姿态图 (Roll, Pitch, Yaw)
figure(handles{3});
subplot(3, 1, 1);
hold on; grid on;
title('Roll Angle');
xlabel('Time (s)');
ylabel('Angle (rad)');
for i = 1:num_agents
    try
        plot(t, actual.att{i}(:, 1), 'Color', colors(i, :), 'LineWidth', 1.5);
        plot(t, desired.att{i}(:, 1), '--', 'Color', colors(i, :), 'LineWidth', 1);
    catch e
        warning('Error plotting attitude Roll for agent %d: %s', i, e.message);
    end
end
legend(legend_str, 'Location', 'best', 'NumColumns', min(num_agents, 3));

subplot(3, 1, 2);
hold on; grid on;
title('Pitch Angle');
xlabel('Time (s)');
ylabel('Angle (rad)');
for i = 1:num_agents
    try
        plot(t, actual.att{i}(:, 2), 'Color', colors(i, :), 'LineWidth', 1.5);
        plot(t, desired.att{i}(:, 2), '--', 'Color', colors(i, :), 'LineWidth', 1);
    catch e
        warning('Error plotting attitude Pitch for agent %d: %s', i, e.message);
    end
end

subplot(3, 1, 3);
hold on; grid on;
title('Yaw Angle');
xlabel('Time (s)');
ylabel('Angle (rad)');
for i = 1:num_agents
    try
        plot(t, actual.att{i}(:, 3), 'Color', colors(i, :), 'LineWidth', 1.5);
        plot(t, desired.att{i}(:, 3), '--', 'Color', colors(i, :), 'LineWidth', 1);
    catch e
        warning('Error plotting attitude Yaw for agent %d: %s', i, e.message);
    end
end

% 角速度图
figure(handles{4});
subplot(3, 1, 1);
hold on; grid on;
title('X Angular Velocity');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
for i = 1:num_agents
    try
        plot(t, actual.omega{i}(:, 1), 'Color', colors(i, :), 'LineWidth', 1.5);
        plot(t, desired.omega{i}(:, 1), '--', 'Color', colors(i, :), 'LineWidth', 1);
    catch e
        warning('Error plotting angular velocity X for agent %d: %s', i, e.message);
    end
end
legend(legend_str, 'Location', 'best', 'NumColumns', min(num_agents, 3));

subplot(3, 1, 2);
hold on; grid on;
title('Y Angular Velocity');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
for i = 1:num_agents
    try
        plot(t, actual.omega{i}(:, 2), 'Color', colors(i, :), 'LineWidth', 1.5);
        plot(t, desired.omega{i}(:, 2), '--', 'Color', colors(i, :), 'LineWidth', 1);
    catch e
        warning('Error plotting angular velocity Y for agent %d: %s', i, e.message);
    end
end

subplot(3, 1, 3);
hold on; grid on;
title('Z Angular Velocity');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
for i = 1:num_agents
    try
        plot(t, actual.omega{i}(:, 3), 'Color', colors(i, :), 'LineWidth', 1.5);
        plot(t, desired.omega{i}(:, 3), '--', 'Color', colors(i, :), 'LineWidth', 1);
    catch e
        warning('Error plotting angular velocity Z for agent %d: %s', i, e.message);
    end
end

% 执行器输出图
figure(handles{5});
subplot(3, 1, 1);
hold on; grid on;
title('Tilt Angle');
xlabel('Time (s)');
ylabel('Angle (rad)');
for i = 1:num_agents
    try
        plot(t, actuator.tilt{i}, 'Color', colors(i, :), 'LineWidth', 1.5);
    catch e
        warning('Error plotting tilt for agent %d: %s', i, e.message);
    end
end
legend(legend_str_single, 'Location', 'best');

subplot(3, 1, 2);
hold on; grid on;
title('Throttle');
xlabel('Time (s)');
ylabel('Throttle');
for i = 1:num_agents
    try
        plot(t, actuator.throttle{i}, 'Color', colors(i, :), 'LineWidth', 1.5);
    catch e
        warning('Error plotting throttle for agent %d: %s', i, e.message);
    end
end

subplot(3, 1, 3);
hold on; grid on;
title('Elevon');
xlabel('Time (s)');
ylabel('Elevon');
for i = 1:num_agents
    try
        plot(t, actuator.elevon{i}, 'Color', colors(i, :), 'LineWidth', 1.5);
    catch e
        warning('Error plotting elevon for agent %d: %s', i, e.message);
    end
end

% 控制矩图
figure(handles{6});
subplot(3, 1, 1);
hold on; grid on;
title('X Control Moment');
xlabel('Time (s)');
ylabel('Moment (N·m)');
for i = 1:num_agents
    try
        plot(t, desired.M{i}(:, 1), 'Color', colors(i, :), 'LineWidth', 1.5);
    catch e
        warning('Error plotting control moment X for agent %d: %s', i, e.message);
    end
end
legend(legend_str_single, 'Location', 'best');

subplot(3, 1, 2);
hold on; grid on;
title('Y Control Moment');
xlabel('Time (s)');
ylabel('Moment (N·m)');
for i = 1:num_agents
    try
        plot(t, desired.M{i}(:, 2), 'Color', colors(i, :), 'LineWidth', 1.5);
    catch e
        warning('Error plotting control moment Y for agent %d: %s', i, e.message);
    end
end

subplot(3, 1, 3);
hold on; grid on;
title('Z Control Moment');
xlabel('Time (s)');
ylabel('Moment (N·m)');
for i = 1:num_agents
    try
        plot(t, desired.M{i}(:, 3), 'Color', colors(i, :), 'LineWidth', 1.5);
    catch e
        warning('Error plotting control moment Z for agent %d: %s', i, e.message);
    end
end

% 三维轨迹图
figure('Name', '3D Trajectory', 'NumberTitle', 'off');
hold on; grid on;
title('3D Trajectory');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
view(3);

% 绘制实际轨迹和期望轨迹
legend_3d = cell(1, 2*num_agents);
for i = 1:num_agents
    try
        % 实际轨迹
        plot3(x{i}(:, 1), x{i}(:, 2), x{i}(:, 3), 'Color', colors(i, :), 'LineWidth', 2);
        % 期望轨迹
        plot3(desired.pos{i}(:, 1), desired.pos{i}(:, 2), desired.pos{i}(:, 3), '--', 'Color', colors(i, :), 'LineWidth', 1);
        
        % 标记起点和终点
        plot3(x{i}(1, 1), x{i}(1, 2), x{i}(1, 3), 'o', 'MarkerSize', 8, 'MarkerFaceColor', colors(i, :), 'MarkerEdgeColor', 'k');
        plot3(x{i}(end, 1), x{i}(end, 2), x{i}(end, 3), 's', 'MarkerSize', 8, 'MarkerFaceColor', colors(i, :), 'MarkerEdgeColor', 'k');
        
        legend_3d{2*i-1} = sprintf('Agent %d Actual', i);
        legend_3d{2*i} = sprintf('Agent %d Desired', i);
    catch e
        warning('Error plotting 3D trajectory for agent %d: %s', i, e.message);
    end
end
legend(legend_3d, 'Location', 'best');

% 调整所有图的大小和位置以便于查看
for i = 1:length(handles)
    if ishandle(handles{i})
        figure(handles{i});
        set(gcf, 'Position', [100 + (i-1)*50, 100 + (i-1)*50, 800, 600]);
    end
end

% 调整轨迹图的大小
figure('Name', '3D Trajectory');
set(gcf, 'Position', [200, 200, 800, 600]);
figure('Name', 'Position Error');
set(gcf, 'Position', [250, 250, 800, 600]);

% 优化所有图的显示
for i = 1:length(handles)
    if ishandle(handles{i})
        figure(handles{i});
        for j = 1:3
            subplot(3, 1, j);
            grid minor;
            ax = gca;
            ax.GridAlpha = 0.3;
            ax.MinorGridAlpha = 0.15;
        end
    end
end

disp('结果绘图完成。');
end