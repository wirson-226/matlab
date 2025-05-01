% modules/plot_group_error.m
function plot_group_error(pos_hist, targets, dt, output_dir)
    [N, ~, T] = size(pos_hist); t = (0:T-1)*dt;
    figure('Name','Group Formation Error','Units','centimeters','Position',[5,5,18,10]);
    for i = 1:N
        e = squeeze(vecnorm(pos_hist(i,:,:) - reshape(targets(i,:),[1,2,1]), 2, 2));
        plot(t, e, 'DisplayName',sprintf('Agent %d',i)); hold on;
    end
    xlabel('Time [s]'); ylabel('||x_i - x_i^*||'); title('Formation Tracking Error'); grid on; legend;
    saveas(gcf, fullfile(output_dir, 'formation_error.eps'), 'epsc');
    print(gcf, fullfile(output_dir, 'formation_error'), '-dpdf');
    print(gcf, fullfile(output_dir, 'formation_error'), '-dpng', '-r300');
end