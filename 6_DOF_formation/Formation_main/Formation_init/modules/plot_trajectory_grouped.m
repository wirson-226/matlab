% modules/plot_trajectory_grouped.m -- 分组目标轨迹显示
function plot_trajectory_grouped(pos_hist, center_hist)
    [N, dim, T] = size(pos_hist);
    [G, ~, ~] = size(center_hist);  % G: 组数
    colororder = lines(N);  % 固定颜色顺序
    group_colors = lines(G);  % 分组中心颜色

    figure('Name','Grouped Trajectories','Units','centimeters','Position',[5,5,18,12]);
    ax = axes; hold(ax, 'on');

    % 画每个智能体的轨迹、起点、终点
    for i = 1:N
        color = colororder(i,:);
        plot(ax, squeeze(pos_hist(i,1,:)), squeeze(pos_hist(i,2,:)), '-', ...
            'LineWidth', 1, 'Color', color, 'DisplayName', sprintf('Agent %d', i));
        scatter(ax, pos_hist(i,1,1), pos_hist(i,2,1), 60, 'o', 'filled', ...
            'MarkerFaceColor', color, 'MarkerEdgeColor', color, 'DisplayName', sprintf('Agent %d Start', i));
        scatter(ax, pos_hist(i,1,end), pos_hist(i,2,end), 120, '^', ...
            'MarkerFaceColor', color, 'MarkerEdgeColor', color, 'DisplayName', sprintf('Agent %d End', i));
    end

    % 绘制每个分组的中心轨迹
    if nargin > 1 && ~isempty(center_hist)
        for g = 1:G
            center_traj = squeeze(center_hist(g, :, :))';
            plot(ax, center_traj(:,1), center_traj(:,2), '--', 'Color', group_colors(g,:), 'LineWidth', 1.5, ...
                'DisplayName', sprintf('Group %d Center Path', g));
            scatter(ax, center_traj(1,1), center_traj(1,2), 100, 'p', 'MarkerEdgeColor', group_colors(g,:), ...
                'MarkerFaceColor', 'none', 'DisplayName', sprintf('Group %d Start', g));
            scatter(ax, center_traj(end,1), center_traj(end,2), 150, 'p', 'MarkerEdgeColor', group_colors(g,:), ...
                'MarkerFaceColor', group_colors(g,:), 'DisplayName', sprintf('Group %d End', g));
        end
    end

    xlabel(ax, 'X [m]'); ylabel(ax, 'Y [m]');
    title(ax, 'Grouped Agent Trajectories and Center Paths');
    axis(ax, 'equal'); grid(ax, 'on');
    legend(ax, 'show', 'Location', 'northeastoutside');
end
