% modules/plot_trajectory.m
function plot_trajectory(pos_hist, output_dir)
    [N, dim, T] = size(pos_hist);
    figure('Name','Trajectories','Units','centimeters','Position',[5,5,16,12]); hold on;
    for i = 1:N
        plot(squeeze(pos_hist(i,1,:)), squeeze(pos_hist(i,2,:)), '-');
        scatter(pos_hist(i,1,1), pos_hist(i,2,1), 40, 'o', 'filled');  % 起点
        scatter(pos_hist(i,1,end), pos_hist(i,2,end), 60, '*');         % 终点
    end
    title('Agent Trajectories'); axis equal; grid on;
    saveas(gcf, fullfile(output_dir, 'trajectories.eps'), 'epsc');
    print(gcf, fullfile(output_dir, 'trajectories'), '-dpdf');
    print(gcf, fullfile(output_dir, 'trajectories'), '-dpng', '-r300');
end