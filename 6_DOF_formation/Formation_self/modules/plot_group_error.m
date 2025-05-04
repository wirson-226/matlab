% modules/plot_group_error.m
% function plot_group_error(pos_hist, targets, dt, ~)
%     [N, ~, T] = size(pos_hist); t = (0:T-1)*dt;
%     figure('Name','Group Formation Error','Units','centimeters','Position',[5,5,18,10]);
%     for i = 1:N
%         e = squeeze(vecnorm(pos_hist(i,:,:) - reshape(targets(i,:),[1,2,1]), 2, 2));
%         plot(t, e, 'DisplayName',sprintf('Agent %d',i)); hold on;
%     end
%     xlabel('Time [s]'); ylabel('||x_i - x_i^*||'); title('Formation Tracking Error'); grid on; legend;
% end

function plot_group_error(pos_hist, targets, dt, ~)
    [N, ~, T] = size(pos_hist); t = (0:T-1)*dt;
    figure('Name','Group Formation Error','Units','centimeters','Position',[5,5,18,10]);

    total_error = zeros(1, T);  % 误差总和曲线

    for i = 1:N
        pos = squeeze(pos_hist(i,:,:))';  % T×2
        if ndims(targets) == 2
            target = repmat(targets(i,:), T, 1);  % 静态目标
        else
            target = squeeze(targets(i,:,:))';    % 动态目标
        end
        e = vecnorm(pos - target, 2, 2);  % T×1
        total_error = total_error + e';   % 误差累加
        plot(t, e, 'DisplayName', sprintf('Agent %d', i)); hold on;
    end

    % 可选择“总误差”或“平均误差”
    % plot(t, total_error, 'k-', 'LineWidth', 2, 'DisplayName', 'Total Error');
    plot(t, total_error / N, 'r--', 'LineWidth', 2, 'DisplayName', 'Avg Error');

    xlabel('Time [s]');
    ylabel('||x_i - x_i^*||');
    title('Formation Tracking Error (with Total)');
    grid on; legend;
end
