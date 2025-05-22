%% 实时状态可视化函数
function fig_handle = plot_formation_state(x, obstacles, obs_radius, t, show_legend)
    % 输入:
    % x - 智能体状态 (4×4): [agent×[x,vx,y,vy]]
    % obstacles - 障碍物位置 (N×2)
    % obs_radius - 障碍物半径
    % t - 当前时间
    % show_legend - 是否显示图例
    
    if nargin < 5
        show_legend = false;
    end
    
    fig_handle = figure(1);
    clf; hold on;
    
    % 设置图形属性
    set(fig_handle, 'Position', [100, 100, 800, 600]);
    
    % === 绘制障碍物 ===
    for i = 1:size(obstacles, 1)
        % 障碍物圆形
        theta = linspace(0, 2*pi, 50);
        obs_x = obstacles(i, 1) + obs_radius * cos(theta);
        obs_y = obstacles(i, 2) + obs_radius * sin(theta);
        fill(obs_x, obs_y, [0.8, 0.3, 0.3], 'FaceAlpha', 0.7, ...
             'EdgeColor', 'red', 'LineWidth', 1.5);
        
        % 障碍物标签
        text(obstacles(i, 1), obstacles(i, 2), sprintf('O%d', i), ...
             'HorizontalAlignment', 'center', 'FontSize', 8, 'Color', 'white', 'FontWeight', 'bold');
    end
    
    % === 绘制Leader (智能体1) ===
    leader_pos = [x(1, 1), x(1, 3)];
    leader_vel = [x(1, 2), x(1, 4)];
    
    % Leader位置（大红圆）
    plot(leader_pos(1), leader_pos(2), 'ro', 'MarkerSize', 15, ...
         'MarkerFaceColor', 'red', 'MarkerEdgeColor', 'black', 'LineWidth', 2);
    
    % Leader速度矢量
    quiver(leader_pos(1), leader_pos(2), leader_vel(1)*0.5, leader_vel(2)*0.5, ...
           'r', 'LineWidth', 2, 'MaxHeadSize', 0.3);
    
    % Leader标签
    text(leader_pos(1)+0.1, leader_pos(2)+0.1, 'L', 'FontSize', 12, ...
         'FontWeight', 'bold', 'Color', 'black');
    
    % === 绘制Followers (智能体2,3,4) ===
    follower_colors = ['b', 'g', 'm'];  % 蓝、绿、品红
    follower_positions = zeros(3, 2);
    
    for i = 2:4
        follower_idx = i - 1;  % follower编号1,2,3
        pos = [x(i, 1), x(i, 3)];
        vel = [x(i, 2), x(i, 4)];
        follower_positions(follower_idx, :) = pos;
        
        % Follower位置（三角形）
        plot(pos(1), pos(2), 's', 'Color', follower_colors(follower_idx), ...
             'MarkerSize', 12, 'MarkerFaceColor', follower_colors(follower_idx), ...
             'MarkerEdgeColor', 'black', 'LineWidth', 1.5);
        
        % Follower速度矢量
        quiver(pos(1), pos(2), vel(1)*0.5, vel(2)*0.5, ...
               follower_colors(follower_idx), 'LineWidth', 1.5, 'MaxHeadSize', 0.3);
        
        % Follower标签
        text(pos(1)+0.1, pos(2)+0.1, sprintf('F%d', follower_idx), ...
             'FontSize', 10, 'FontWeight', 'bold', 'Color', follower_colors(follower_idx));
    end
    
    % === 绘制编队形状 ===
    % 连接三个follower形成三角形
    triangle_x = [follower_positions(:, 1); follower_positions(1, 1)];
    triangle_y = [follower_positions(:, 2); follower_positions(1, 2)];
    plot(triangle_x, triangle_y, 'k--', 'LineWidth', 1.5, 'Color', [0.5, 0.5, 0.5]);
    
    % 计算编队中心
    formation_center = mean(follower_positions, 1);
    plot(formation_center(1), formation_center(2), 'ko', 'MarkerSize', 8, ...
         'MarkerFaceColor', 'yellow', 'MarkerEdgeColor', 'black');
    
    % === 绘制参考轨迹 ===
    % Leader参考轨迹（红色虚线圆）
    w_lead = 0.157; r_lead = 0.3;
    theta_ref = linspace(0, 2*pi, 100);
    ref_x = r_lead * cos(theta_ref);
    ref_y = r_lead * sin(theta_ref);
    plot(ref_x, ref_y, 'r:', 'LineWidth', 1, 'Color', [1, 0.5, 0.5]);
    
    % Formation参考轨迹（灰色虚线圆）
    w = 0.314; r = 0.5;
    ref_x_form = r * cos(theta_ref);
    ref_y_form = r * sin(theta_ref);
    plot(ref_x_form, ref_y_form, ':', 'LineWidth', 1, 'Color', [0.7, 0.7, 0.7]);
    
    % === 图形设置 ===
    axis equal;
    grid on;
    xlim([-4, 4]);
    ylim([-4, 4]);
    xlabel('X Position (m)', 'FontSize', 12);
    ylabel('Y Position (m)', 'FontSize', 12);
    title(sprintf('Three-Follower Formation Control at t = %.2f s', t), 'FontSize', 14);
    
    % 添加图例
    if show_legend
        legend_items = {
            'Obstacles', 'Leader', 'Leader Velocity', 'Follower 1', 'Follower 2', 'Follower 3', ...
            'Formation Shape', 'Formation Center', 'Leader Reference', 'Formation Reference'
        };
        legend(legend_items, 'Location', 'northeastoutside', 'FontSize', 10);
    end
    
    % 添加信息文本框
    info_text = sprintf('t = %.2f s\nLeader: (%.2f, %.2f)\nFormation Center: (%.2f, %.2f)', ...
                       t, leader_pos(1), leader_pos(2), formation_center(1), formation_center(2));
    text(-3.8, 3.5, info_text, 'FontSize', 10, 'BackgroundColor', 'white', ...
         'EdgeColor', 'black', 'VerticalAlignment', 'top');
    
    drawnow;
end