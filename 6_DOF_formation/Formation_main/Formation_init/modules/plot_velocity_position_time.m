% modules/plot_velocity_position_time.m -- 优化版
function plot_velocity_position_time(state_hist, dt, ~)
% 输入：state_hist - [steps, num_agents, n_dim] 状态历史
% dt - 时间步长
    [steps, num_agents, n_dim] = size(state_hist);
    t = (0:steps-1) * dt; % 时间向量
    colororder = lines(num_agents);

    % Position plot with tiledlayout
    figure('Name','Position vs Time','Units','centimeters','Position',[5,5,18,10]);
    tl1 = tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

    % 绘制所有智能体的位置
    for i = 1:num_agents
        % 设置图例标签和线条样式：第一个为领导者，其余为跟随者
        if i == 1
            agent_label = 'Leader';
            line_color = [0, 0, 1]; % 蓝色
            line_width = 1.3; % 更粗
        else
            agent_label = sprintf('Follower %d', i-1);
            line_color = colororder(i,:);
            line_width = 1.2;
        end
        
        % X位置
        nexttile(1); hold on;
        plot(t, state_hist(:, i, 1), 'Color', line_color, ...
            'LineWidth', line_width, 'DisplayName', agent_label);
        
        % Y位置
        nexttile(2); hold on;
        plot(t, state_hist(:, i, 3), 'Color', line_color, ...
            'LineWidth', line_width, 'DisplayName', agent_label);
        
        % 位置大小（距离原点的距离）
        nexttile(3); hold on;
        pos_magnitude = sqrt(state_hist(:, i, 1).^2 + state_hist(:, i, 3).^2);
        plot(t, pos_magnitude, 'Color', line_color, ...
            'LineWidth', line_width, 'DisplayName', agent_label);
    end

    % 设置位置图的标题和标签
    nexttile(1);
    title('X Position vs Time');
    grid on;
    ylabel('X Position [m]');
    legend('show','Location','northeastoutside');

    nexttile(2);
    title('Y Position vs Time');
    grid on;
    ylabel('Y Position [m]');
    legend('show','Location','northeastoutside');

    nexttile(3);
    title('Position Magnitude vs Time');
    grid on;
    xlabel('Time [s]');
    ylabel('Distance from Origin [m]');
    legend('show','Location','northeastoutside');

    % Velocity plot with tiledlayout
    figure('Name','Velocity vs Time','Units','centimeters','Position',[25,5,18,10]);
    tl2 = tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

    % 绘制所有智能体的速度
    for i = 1:num_agents
        % 设置图例标签和线条样式：第一个为领导者，其余为跟随者
        if i == 1
            agent_label = 'Leader';
            line_color = [0, 0, 1]; % 绿色
            line_width = 1.3; % 更粗
        else
            agent_label = sprintf('Follower %d', i-1);
            line_color = colororder(i,:);
            line_width = 1.2;
        end
        
        % X速度
        nexttile(1); hold on;
        plot(t, state_hist(:, i, 2), 'Color', line_color, ...
            'LineWidth', line_width, 'DisplayName', agent_label);
        
        % Y速度
        nexttile(2); hold on;
        plot(t, state_hist(:, i, 4), 'Color', line_color, ...
            'LineWidth', line_width, 'DisplayName', agent_label);
        
        % 速度大小
        nexttile(3); hold on;
        vel_magnitude = sqrt(state_hist(:, i, 2).^2 + state_hist(:, i, 4).^2);
        plot(t, vel_magnitude, 'Color', line_color, ...
            'LineWidth', line_width, 'DisplayName', agent_label);
    end

    % 设置速度图的标题和标签
    nexttile(1);
    title('X Velocity vs Time');
    grid on;
    ylabel('X Velocity [m/s]');
    legend('show','Location','northeastoutside');

    nexttile(2);
    title('Y Velocity vs Time');
    grid on;
    ylabel('Y Velocity [m/s]');
    legend('show','Location','northeastoutside');

    nexttile(3);
    title('Velocity Magnitude vs Time');
    grid on;
    xlabel('Time [s]');
    ylabel('Speed [m/s]');
    legend('show','Location','northeastoutside');
end