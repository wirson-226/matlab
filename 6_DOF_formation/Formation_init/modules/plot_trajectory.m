% modules/plot_trajectory.m -- 带目标轨迹显示
function plot_trajectory(pos_hist, center_hist)
    [N, dim, T] = size(pos_hist);
    colororder = lines(N);  % 固定颜色顺序
    t_centers = size(center_hist, 1);

    figure('Name','Trajectories','Units','centimeters','Position',[5,5,18,12]);
    ax = axes; hold(ax, 'on');

    % 画每个智能体的轨迹、起点、终点
    for i = 1:N
        color = colororder(i,:);
        plot(ax, squeeze(pos_hist(i,1,:)), squeeze(pos_hist(i,2,:)), '-', ...
            'LineWidth', 1, 'Color', color, 'DisplayName', sprintf('Agent %d', i));  % 智能体轨迹线
        scatter(ax, pos_hist(i,1,1), pos_hist(i,2,1), 60, 'o', 'filled', ...
            'MarkerFaceColor', color, 'MarkerEdgeColor', color, 'DisplayName', sprintf('Agent %d Start', i));   % 智能体起点
        scatter(ax, pos_hist(i,1,end), pos_hist(i,2,end), 120, '^', ...
            'MarkerFaceColor', color, 'MarkerEdgeColor', color, 'DisplayName', sprintf('Agent %d End', i));  % 智能体终点
    end

    % 绘制中心轨迹
    if nargin > 1 && ~isempty(center_hist)
        plot(center_hist(:,1), center_hist(:,2), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Center Path');  % 跟踪目标轨迹线
        scatter(center_hist(1,1), center_hist(1,2), 100, 'p', ...
            'MarkerEdgeColor','k', 'MarkerFaceColor','none', 'DisplayName', 'Center Start');  % 跟踪目标起点
        scatter(center_hist(end,1), center_hist(end,2), 150, 'p', ...
            'MarkerEdgeColor','k', 'MarkerFaceColor','y', 'DisplayName', 'Center End');     % 跟踪目标终点
    end

    xlabel(ax, 'X [m]'); ylabel(ax, 'Y [m]');
    title(ax, 'Agent Trajectories and Tracking Path');
    axis(ax, 'equal'); grid(ax, 'on');

    % 简洁图例（仅一部分元素）
    legend(ax, 'show', 'Location', 'northeastoutside');
end
