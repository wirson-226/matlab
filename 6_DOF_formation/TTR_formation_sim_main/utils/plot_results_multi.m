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
    

%% Position (X, Y, Z) - Multi-agent with Correct Legend Binding
figure('Name', 'Position', 'Units', 'centimeters', 'Position', [5, 5, 42, 20]);
tiledlayout(3,1, 'TileSpacing','compact', 'Padding','compact');

titles  = {'X Position', 'Y Position', 'Z Position'};
ylabels = {'X[m]', 'Y[m]', 'Z[m]'};

legend_entries = cell(1, 2 * num_agents);     % 图例内容
legend_handles = gobjects(1, 2 * num_agents); % 图线句柄（必须绑定）

for k = 1:3
    ax = nexttile;
    hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');

    for i = 1:num_agents
        try
            % 实际与期望曲线 + 获取句柄
            h1 = plot(ax, t, x{i}(:,k), '-',  'Color', colors(i,:), 'LineWidth', 1.5);
            h2 = plot(ax, t, desired.pos{i}(:,k), '--', 'Color', colors(i,:), 'LineWidth', 1);

            % 只在第一个子图收集 legend 内容与句柄
            if k == 1
                legend_entries{2*i - 1} = sprintf('Agent %d Actual', i);
                legend_entries{2*i}     = sprintf('Agent %d Desired', i);
                legend_handles(2*i - 1) = h1;
                legend_handles(2*i)     = h2;
            end
        catch e
            warning('Error plotting position dim %d for agent %d: %s', k, i, e.message);
        end
    end

    title(ax, titles{k}, 'Interpreter', 'latex');
    xlabel(ax, 'Time [s]', 'Interpreter', 'latex');
    ylabel(ax, ylabels{k}, 'Interpreter', 'latex');
    set(ax, 'FontName', 'Times New Roman', 'FontSize', 12, 'LineWidth', 1);
end

% ✅ Legend 显示绑定句柄，确保不遗漏任何曲线
legend(nexttile(2), legend_handles, legend_entries, ...
       'Location', 'eastoutside', 'Box', 'off', ...
       'FontSize', 10, 'Interpreter', 'latex');


%% Position Error (X, Y, Z) - Multi-agent Optimized
figure('Name', 'Position Error', 'Units', 'centimeters', 'Position', [5, 5, 42, 20]);
tiledlayout(3,1, 'TileSpacing','compact', 'Padding','compact');

titles  = {'X Position Error', 'Y Position Error', 'Z Position Error'};
ylabels = {'X error[m]', 'Y error[m]', 'Z error[m]'};

for k = 1:3
    ax = nexttile;
    hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
    title(ax, titles{k}, 'Interpreter', 'latex');
    xlabel(ax, 'Time [s]', 'Interpreter', 'latex');
    ylabel(ax, ylabels{k}, 'Interpreter', 'latex');

    legend_entries = cell(1, num_agents);
    for i = 1:num_agents
        try
            plot(ax, x{i}(:,k) - desired.pos{i}(:,k), ...
                 'Color', colors(i,:), 'LineWidth', 1.5);
            legend_entries{i} = sprintf('Agent %d', i);
        catch e
            warning('Error plotting position error %d for agent %d: %s', k, i, e.message);
        end
    end

    legend(ax, legend_entries, ...
           'Location', 'eastoutside', 'Box', 'off', ...
           'FontSize', 10, 'Interpreter', 'latex');

    set(ax, 'FontName', 'Times New Roman', 'FontSize', 12, 'LineWidth', 1);
end



%% Velocity (Vx, Vy, Vz) - Multi-agent Optimized with Legend Binding
figure('Name', 'Velocity', 'Units', 'centimeters', 'Position', [5, 5, 42, 20]);
tiledlayout(3,1, 'TileSpacing','compact', 'Padding','compact');

titles  = {'X Velocity', 'Y Velocity', 'Z Velocity'};
ylabels = {'Vx [m/s]', 'Vy [m/s]', 'Vz [m/s]'};

legend_entries = cell(1, 2 * num_agents);
legend_handles = gobjects(1, 2 * num_agents);

for k = 1:3
    ax = nexttile;
    hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');

    for i = 1:num_agents
        try
            h1 = plot(ax, t, x{i}(:, k+3), '-',  'Color', colors(i,:), 'LineWidth', 1.5);
            h2 = plot(ax, t, desired.vel{i}(:, k), '--', 'Color', colors(i,:), 'LineWidth', 1);

            if k == 1
                legend_entries{2*i - 1} = sprintf('Agent %d Actual', i);
                legend_entries{2*i}     = sprintf('Agent %d Desired', i);
                legend_handles(2*i - 1) = h1;
                legend_handles(2*i)     = h2;
            end
        catch e
            warning('Error plotting velocity dim %d for agent %d: %s', k, i, e.message);
        end
    end

    title(ax, titles{k}, 'Interpreter', 'latex');
    xlabel(ax, 'Time [s]', 'Interpreter', 'latex');
    ylabel(ax, ylabels{k}, 'Interpreter', 'latex');
    set(ax, 'FontName', 'Times New Roman', 'FontSize', 12, 'LineWidth', 1);
end

% ✅ 只在第一个 subplot 添加图例（绑定句柄）
legend(nexttile(2), legend_handles, legend_entries, ...
       'Location', 'eastoutside', 'Box', 'off', ...
       'FontSize', 10, 'Interpreter', 'latex');

%% Attitude (Roll, Pitch, Yaw) - Multi-agent with Legend Binding
figure('Name', 'Attitude', 'Units', 'centimeters', 'Position', [5, 5, 42, 20]);
tiledlayout(3,1, 'TileSpacing','compact', 'Padding','compact');

titles  = {'Roll Angle', 'Pitch Angle', 'Yaw Angle'};
ylabels = {'\phi[rad]', '\theta[rad]', '\psi[rad]'};

legend_entries = cell(1, 2 * num_agents);
legend_handles = gobjects(1, 2 * num_agents);

for k = 1:3
    ax = nexttile;
    hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');

    for i = 1:num_agents
        try
            h1 = plot(ax, t, actual.att{i}(:, k), '-',  'Color', colors(i,:), 'LineWidth', 1.5);
            h2 = plot(ax, t, desired.att{i}(:, k), '--', 'Color', colors(i,:), 'LineWidth', 1);

            if k == 1
                legend_entries{2*i - 1} = sprintf('Agent %d Actual', i);
                legend_entries{2*i}     = sprintf('Agent %d Desired', i);
                legend_handles(2*i - 1) = h1;
                legend_handles(2*i)     = h2;
            end
        catch e
            warning('Error plotting attitude %d for agent %d: %s', k, i, e.message);
        end
    end

    title(ax, titles{k});
    xlabel(ax, 'Time [s]');
    ylabel(ax, ylabels{k});
    set(ax, 'FontName', 'Times New Roman', 'FontSize', 12, 'LineWidth', 1);
end

legend(nexttile(2), legend_handles, legend_entries, ...
       'Location', 'eastoutside', 'Box', 'off', ...
       'FontSize', 10, 'Interpreter', 'latex');
% %% Angular Velocity (p, q, r) - Multi-agent with Legend Binding
figure('Name', 'Angular Velocity', 'Units', 'centimeters', 'Position', [5, 5, 42, 20]);
tiledlayout(3,1, 'TileSpacing','compact', 'Padding','compact');

titles  = {'X Angular Velocity', 'Y Angular Velocity', 'Z Angular Velocity'};
ylabels = {'p [rad/s]', 'q [rad/s]', 'r [rad/s]'};

legend_entries = cell(1, 2 * num_agents);
legend_handles = gobjects(1, 2 * num_agents);

for k = 1:3
    ax = nexttile;
    hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');

    for i = 1:num_agents
        try
            h1 = plot(ax, t, actual.omega{i}(:, k), '-',  'Color', colors(i,:), 'LineWidth', 1.5);
            h2 = plot(ax, t, desired.omega{i}(:, k), '--', 'Color', colors(i,:), 'LineWidth', 1);

            if k == 1
                legend_entries{2*i - 1} = sprintf('Agent %d Actual', i);
                legend_entries{2*i}     = sprintf('Agent %d Desired', i);
                legend_handles(2*i - 1) = h1;
                legend_handles(2*i)     = h2;
            end
        catch e
            warning('Error plotting angular velocity %d for agent %d: %s', k, i, e.message);
        end
    end

    title(ax, titles{k}, 'Interpreter', 'latex');
    xlabel(ax, 'Time [s]', 'Interpreter', 'latex');
    ylabel(ax, ylabels{k}, 'Interpreter', 'latex');
    set(ax, 'FontName', 'Times New Roman', 'FontSize', 12, 'LineWidth', 1);
end

legend(nexttile(2), legend_handles, legend_entries, ...
       'Location', 'eastoutside', 'Box', 'off', ...
       'FontSize', 10, 'Interpreter', 'latex');



%% Actuator Outputs (Tilt / Throttle / Elevon) - Multi-agent Optimized
figure('Name', 'Actuator Outputs', 'Units', 'centimeters', 'Position', [5, 5, 42, 20]);
tiledlayout(3,1, 'TileSpacing','compact', 'Padding','compact');

titles   = {'Tilt Angle', 'Throttle', 'Elevon'};
ylabels  = {'Tilt Angle', 'Throttle', 'Elevon'};
fields   = {'tilt', 'throttle', 'elevon'};

for k = 1:3
    ax = nexttile;
    hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
    title(ax, titles{k}, 'Interpreter', 'latex');
    xlabel(ax, 'Time (s)', 'Interpreter', 'latex');
    ylabel(ax, ylabels{k}, 'Interpreter', 'latex');

    legend_entries = cell(1, num_agents);
    for i = 1:num_agents
        try
            plot(ax, t, actuator.(fields{k}){i}, ...
                 'Color', colors(i,:), 'LineWidth', 1.5);
            legend_entries{i} = sprintf('Agent %d', i);
        catch e
            warning('Error plotting %s for agent %d: %s', fields{k}, i, e.message);
        end
    end

    legend(ax, legend_entries, 'Location', 'eastoutside', ...
           'Box', 'off', 'FontSize', 10, 'Interpreter', 'latex');
    set(ax, 'FontName', 'Times New Roman', 'FontSize', 12, 'LineWidth', 1);
end



%% Control Moments (Mx, My, Mz) - Multi-agent Optimized
figure('Name', 'Control Moments', 'Units', 'centimeters', 'Position', [5, 5, 42, 20]);
tiledlayout(3,1, 'TileSpacing','compact', 'Padding','compact');

titles  = {'X Control Moment', 'Y Control Moment', 'Z Control Moment'};
ylabels = {'My [Nm]', 'Mx [Nm]', 'Mz [Nm]'};

for k = 1:3
    ax = nexttile;
    hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');

    legend_entries = cell(1, num_agents);
    for i = 1:num_agents
        try
            plot(ax, t, desired.M{i}(:,k), ...
                 'Color', colors(i,:), 'LineWidth', 1.5);
            legend_entries{i} = sprintf('Agent %d', i);
        catch e
            warning('Error plotting control moment %d for agent %d: %s', k, i, e.message);
        end
    end

    title(ax, titles{k});
    xlabel(ax, 'Time [s]');
    ylabel(ylabels{k});

    legend(ax, legend_entries, 'Location', 'eastoutside', ...
           'Box', 'off', 'FontSize', 10, 'Interpreter', 'latex');
    set(ax, 'FontName', 'Times New Roman', 'FontSize', 12, 'LineWidth', 1);
end

%% 3D Trajectory - Multi-agent Optimized
figure('Name', '3D Trajectory', 'Units', 'centimeters', 'Position', [5, 5, 42, 24]);
hold on; grid on; box on;
view(3);
title('3D Trajectory', 'Interpreter', 'latex');
xlabel('X (m)', 'Interpreter', 'latex');
ylabel('Y (m)', 'Interpreter', 'latex');
zlabel('Z (m)', 'Interpreter', 'latex');

legend_entries = cell(1, 2 * num_agents);  % 提前分配更高效

for i = 1:num_agents
    try
        % 实际轨迹
        plot3(x{i}(:,1), x{i}(:,2), x{i}(:,3), '-', 'Color', colors(i,:), 'LineWidth', 2);
        % 期望轨迹
        plot3(desired.pos{i}(:,1), desired.pos{i}(:,2), desired.pos{i}(:,3), '--', 'Color', colors(i,:), 'LineWidth', 1);

        % 起点与终点标记
        plot3(x{i}(1,1), x{i}(1,2), x{i}(1,3), 'o', ...
              'MarkerSize', 6, 'MarkerFaceColor', colors(i,:), 'MarkerEdgeColor', 'k');
        plot3(x{i}(end,1), x{i}(end,2), x{i}(end,3), 's', ...
              'MarkerSize', 6, 'MarkerFaceColor', colors(i,:), 'MarkerEdgeColor', 'k');

        % 图例绑定
        legend_entries{2*i-1} = sprintf('Agent %d Actual', i);
        legend_entries{2*i}   = sprintf('Agent %d Desired', i);
    catch e
        warning('Error plotting trajectory for agent %d: %s', i, e.message);
    end
end

legend(legend_entries, 'Location', 'eastoutside', ...
       'FontSize', 10, 'Interpreter', 'latex', 'Box', 'off');
set(gca, 'FontName', 'Times New Roman', 'FontSize', 12, 'LineWidth', 1);



end

