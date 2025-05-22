% % modules/plot_trajectory.m -- 带目标轨迹显示（优化版）
% function plot_trajectory(state_hist, static_obs)
function plot_trajectory(state_hist, static_obs)
    % 输入：state_hist - [steps, num_agents, n_dim] 状态历史
    %      static_obs - 静态障碍物位置 [num_obs, 2]
    
    [steps, num_agents, n_dim] = size(state_hist);
    
    % 提取位置历史：[steps, num_agents, 2] (只取x和y坐标)
    pos_hist = state_hist(:, :, [1, 3]); % 提取x和y坐标列
    
    colororder = lines(num_agents); % 固定颜色顺序
    
    % 创建图形窗口
    figure('Name','Trajectories','Units','centimeters','Position',[5,5,18,12]);
    ax = axes; 
    hold(ax, 'on');
    
    % 绘制跟随者轨迹、起点、终点
    for i = 2:num_agents
        color = colororder(i,:);
        
        % 提取第i个智能体的轨迹
        traj_x = pos_hist(:, i, 1); % x坐标
        traj_y = pos_hist(:, i, 2); % y坐标
        
        % 绘制轨迹线
        plot(ax, traj_x, traj_y, '-', ...
            'LineWidth', 1, 'Color', color, ...
            'DisplayName', sprintf('Agent %d', i));
        
        % 绘制起点
        scatter(ax, traj_x(1), traj_y(1), 60, 'o', 'filled', ...
            'MarkerFaceColor', color, 'MarkerEdgeColor', color, ...
            'DisplayName', sprintf('Agent %d Start', i));
        
        % 绘制终点
        scatter(ax, traj_x(end), traj_y(end), 120, '^', ...
            'MarkerFaceColor', color, 'MarkerEdgeColor', color, ...
            'DisplayName', sprintf('Agent %d End', i));
    end
    

    % 绘制静态障碍物
    if ~isempty(static_obs)
        for i = 1:size(static_obs, 1)
            theta = linspace(0, 2*pi, 50);
            obs_x = static_obs(i, 1) + 0.3 * cos(theta);
            obs_y = static_obs(i, 2) + 0.3 * sin(theta);
            
            if i == 1
                % 第一个障碍物显示在图例中
                fill(obs_x, obs_y, [0.8, 0.3, 0.3], ...
                    'FaceAlpha', 0.5, 'DisplayName', 'Obstacles');
            else
                % 其他障碍物不占图例
                fill(obs_x, obs_y, [0.8, 0.3, 0.3], ...
                    'FaceAlpha', 0.5, 'HandleVisibility', 'off');
            end
        end
    end

    
    % 绘制领导者轨迹（智能体1）
    leader_x = pos_hist(:, 1, 1); % 领导者x坐标
    leader_y = pos_hist(:, 1, 2); % 领导者y坐标
    
    plot(ax, leader_x, leader_y, 'k--', 'LineWidth', 1.5, ...
        'DisplayName', 'Leader Path');
    
    % 领导者起点
    scatter(ax, leader_x(1), leader_y(1), 100, 'p', ...
        'MarkerEdgeColor','k', 'MarkerFaceColor','none', ...
        'DisplayName', 'Leader Start');
    
    % 领导者终点
    scatter(ax, leader_x(end), leader_y(end), 150, 'p', ...
        'MarkerEdgeColor','k', 'MarkerFaceColor','y', ...
        'DisplayName', 'Leader End');
    
    % 设置图形属性
    xlabel(ax, 'X [m]'); 
    ylabel(ax, 'Y [m]');
    title(ax, 'Multi-Agent Trajectories and Leader Path');
    axis(ax, 'equal'); 
    grid(ax, 'on');

    legend(ax, 'show', 'Location', 'northeastoutside');
end



