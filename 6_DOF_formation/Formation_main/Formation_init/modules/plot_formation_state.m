function fig_handle = plot_formation_state(x, obstacles, obs_radius, t, show_legend, ctrl)

    % 创建图形
    fig_handle = figure(1);
    clf; hold on;
    set(fig_handle, 'Position', [100, 100, 800, 600]);
    N = size(x,1); % 获取智能体+跟随者数量

    % ===== 1. 全局字体设置 =====
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 12);
    velocity_scale = 1.0; % 调节此值改变所有箭头长度

    % ===== 2. 绘制障碍物 =====
    for i = 1:size(obstacles, 1)
        theta = linspace(0, 2*pi, 50);
        obs_x = obstacles(i, 1) + obs_radius * cos(theta);
        obs_y = obstacles(i, 2) + obs_radius * sin(theta);
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

    % ===== 3. 绘制Leader (五角星) =====
    leader_pos = [x(1, 1), x(1, 3)];
    leader_vel = [x(1, 2), x(1, 4)];

    % Leader位置（红色五角星）
    plot(leader_pos(1), leader_pos(2), 'p', 'Color', 'green', ...
         'MarkerSize', 18, 'MarkerFaceColor', 'green', ...
         'MarkerEdgeColor', 'black', 'LineWidth', 1.5, 'DisplayName', 'Leader');

    % Leader速度矢量
    quiver(leader_pos(1), leader_pos(2), leader_vel(1)*velocity_scale, leader_vel(2)*velocity_scale, ...
           'r', 'LineWidth', 1.5, 'MaxHeadSize', 3, 'DisplayName', 'Leader Velocity');

    % ===== 4. 绘制Followers (三角形) =====
    follower_colors = [0 0.45 0.74; 0.85 0.33 0.1; 0.93 0.69 0.13]; % 蓝/橙/黄
    follower_positions = zeros(3, 2);

    for i = 2:N
        follower_idx = i-1;
        pos = [x(i, 1), x(i, 3)];
        vel = [x(i, 2), x(i, 4)];
        follower_positions(follower_idx, :) = pos;

        % Follower位置（三角形）
        plot(pos(1), pos(2), '^', 'Color', follower_colors(follower_idx,:), ...
             'MarkerSize', 12, 'MarkerFaceColor', follower_colors(follower_idx,:), ...
             'MarkerEdgeColor', 'black', 'LineWidth', 1, ...
             'DisplayName', sprintf('Follower %d', follower_idx));

        % 调节方案1：固定缩放因子
        quiver(pos(1), pos(2), vel(1)*velocity_scale, vel(2)*velocity_scale, ...
               'Color', follower_colors(follower_idx,:), ...
               'LineWidth', 2, ...
               'MaxHeadSize', 3, ...
               'AutoScale', 'off', ... % 关键！关闭自动缩放
               'DisplayName', sprintf('F%d Velocity', follower_idx));
    end  
    % ===== 5. 编队框架绘制 =====
    % 当前编队形状（灰色虚线三角形）
    plot([follower_positions(:,1); follower_positions(1,1)], ...
         [follower_positions(:,2); follower_positions(1,2)], ...
         '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1, 'DisplayName', 'Current Formation');

    % 理想编队圆形框架（蓝色虚线）
    formation_center = mean(follower_positions);
    theta = linspace(0, 2*pi, 100);
    ideal_x = formation_center(1) + ctrl.r * cos(theta);
    ideal_y = formation_center(2) + ctrl.r * sin(theta);
    plot(ideal_x, ideal_y, ':', 'Color', [0 0.45 0.74], 'LineWidth', 1.5, ...
         'DisplayName', 'Follower reference');

    % ===== 6. 参考轨迹 =====
    % Leader参考轨迹（红色虚线圆）
    theta_ref = linspace(0, 2*pi, 100);
    ref_x = ctrl.r_lead * cos(theta_ref);
    ref_y = ctrl.r_lead * sin(theta_ref);
    plot(ref_x, ref_y, '-', 'Color', 'green', 'LineWidth', 1, ...
         'DisplayName', 'Leader Reference');

    % ===== 7. 图形美化 =====
    axis equal; grid on;
    xlim([-4, 4]); ylim([-4, 4]);
    xlabel('X Position (m)', 'FontSize', 12, 'FontName', 'Times New Roman');
    ylabel('Y Position (m)', 'FontSize', 12, 'FontName', 'Times New Roman');
    title(sprintf('Formation Control at t = %.2f s', t), ...
          'FontSize', 14, 'FontName', 'Times New Roman', 'FontWeight', 'bold');

    % ===== 8. 智能图例 =====
    if show_legend
        % 获取所有带DisplayName的图形对象
        h = findobj(gca, '-property', 'DisplayName');
        legend_entries = arrayfun(@(x) get(x, 'DisplayName'), h, 'UniformOutput', false);

        % 去除重复项
        [~, unique_idx] = unique(legend_entries, 'stable');
        legend_handle = legend(h(unique_idx), ...
               'Location', 'northeastoutside', ...
               'FontName', 'Times New Roman', ...
               'FontSize', 16, ...
               'Box', 'off');
        % 手动设置图例位置确保在外面
        legend_pos = get(legend_handle, 'Position');
        legend_pos(1) = 0.78;  % 手动设置x位置
        legend_pos(2) = 0.3;   % 手动设置y位置
        set(legend_handle, 'Position', legend_pos);
    end

    % ===== 9. 信息标注 =====
    info_text = sprintf('\\fontname{Times New Roman}\\fontsize{10}t = %.2f s\\fontsize{9}\\newlineLeader: (%.2f, %.2f)\\newlineFormation Radius: %.2f m', ...
                       t, leader_pos(1), leader_pos(2), ctrl.r);
    annotation('textbox', [0.02, 0.85, 0.2, 0.1], ...
               'String', info_text, ...
               'FontName', 'Times New Roman', ...
               'EdgeColor', [0.6 0.6 0.6], ...
               'BackgroundColor', [1 1 1 0.7], ...
               'FitBoxToText', 'on');

    drawnow;
end

