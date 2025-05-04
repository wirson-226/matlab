% modules/plot_trajectory.m -- 避免图例占用空间
function plot_trajectory(pos_hist, ~)
    [N, dim, T] = size(pos_hist);
    colororder = lines(N);  % 固定颜色顺序

    figure('Name','Trajectories');
    ax = axes; hold(ax, 'on');
    for i = 1:N
        color = colororder(i,:);
        plot(ax, squeeze(pos_hist(i,1,:)), squeeze(pos_hist(i,2,:)), '-', 'LineWidth', 1, 'Color', color, 'DisplayName', sprintf('Agent %d', i));  % 轨迹线
        scatter(ax, pos_hist(i,1,1), pos_hist(i,2,1), 60, 'o', 'filled', 'MarkerFaceColor', color, 'MarkerEdgeColor', color, 'DisplayName', sprintf('Agent %d Start', i));   % 起点
        scatter(ax, pos_hist(i,1,end), pos_hist(i,2,end), 120, '^', 'MarkerFaceColor', color, 'MarkerEdgeColor', color, 'DisplayName', sprintf('Agent %d End', i));  % 终点
    end
    xlabel(ax, 'X [m]'); ylabel(ax, 'Y [m]');
    title(ax, 'Agent Trajectories'); axis(ax, 'equal'); grid(ax, 'on');
    legend(ax, 'show', 'Location', 'northeastoutside');
end
