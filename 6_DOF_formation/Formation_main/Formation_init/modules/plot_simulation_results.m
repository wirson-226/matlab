%% 仿真结果分析函数（更新版）
function plot_simulation_results(state_hist, center_hist, static_obs, dt)
    steps = size(state_hist, 1);
    num_agents = size(state_hist, 2);
    time_vec = (0:steps-1) * dt;

    figure(2);
    set(gcf, 'Position', [200, 200, 1200, 800]);

    % === 子图1: 轨迹图 ===
    subplot(2, 3, 1);
    hold on;
    for i = 1:num_agents
        traj_x = squeeze(state_hist(:, i, 1));
        traj_y = squeeze(state_hist(:, i, 3));
        plot(traj_x, traj_y, 'LineWidth', 1.5);
        plot(traj_x(1), traj_y(1), 'o', 'MarkerSize', 5, 'MarkerFaceColor', 'g');
        plot(traj_x(end), traj_y(end), 's', 'MarkerSize', 5, 'MarkerFaceColor', 'r');
    end
    for i = 1:size(static_obs, 1)
        theta = linspace(0, 2*pi, 50);
        obs_x = static_obs(i, 1) + 0.3 * cos(theta);
        obs_y = static_obs(i, 2) + 0.3 * sin(theta);
        fill(obs_x, obs_y, [0.8, 0.3, 0.3], 'FaceAlpha', 0.5);
    end
    plot(center_hist(:, 1), center_hist(:, 2), 'k--', 'LineWidth', 1);
    axis equal; grid on;
    xlabel('X (m)'); ylabel('Y (m)');
    title('Agent Trajectories');

    % === 子图2: 编队误差 ===
    subplot(2, 3, 2);
    formation_error = zeros(steps, 1);
    for step = 1:steps
        d_sum = 0; count = 0;
        for i = 1:num_agents
            for j = i+1:num_agents
                pos_i = squeeze(state_hist(step, i, [1, 3]));
                pos_j = squeeze(state_hist(step, j, [1, 3]));
                d_sum = d_sum + abs(norm(pos_i - pos_j) - 1.0);
                count = count + 1;
            end
        end
        formation_error(step) = d_sum / count;
    end
    plot(time_vec, formation_error, 'b-', 'LineWidth', 2);
    grid on;
    xlabel('Time (s)'); ylabel('Formation Error (m)');
    title('Formation Structure Error');

    % === 子图3: 每个智能体位置随时间变化 (X) ===
    subplot(2, 3, 3);
    hold on;
    for i = 1:num_agents
        pos_x = squeeze(state_hist(:, i, 1));
        plot(time_vec, pos_x, 'LineWidth', 1.2);
    end
    xlabel('Time (s)'); ylabel('X Position (m)');
    title('Agent Position vs Time');
    grid on;

    % === 子图4: 速度分析 ===
    subplot(2, 3, 4);
    for i = 1:num_agents
        vel_x = squeeze(state_hist(:, i, 2));
        vel_y = squeeze(state_hist(:, i, 4));
        vel_mag = sqrt(vel_x.^2 + vel_y.^2);
        plot(time_vec, vel_mag, 'LineWidth', 1.2);
        hold on;
    end
    xlabel('Time (s)'); ylabel('Velocity (m/s)');
    title('Agent Velocities');
    grid on;

    % === 子图5: 所有智能体轨迹 ===
    subplot(2, 3, 5);
    hold on;
    for i = 1:num_agents
        traj_x = squeeze(state_hist(:, i, 1));
        traj_y = squeeze(state_hist(:, i, 3));
        plot(traj_x, traj_y, 'LineWidth', 1.2);
    end
    axis equal; grid on;
    xlabel('X (m)'); ylabel('Y (m)');
    title('All Agent Trajectories');

    % === 子图6: 性能柱状图 ===
    subplot(2, 3, 6);
    final_form_err = formation_error(end);
    mean_form_err = mean(formation_error);
    metrics = [final_form_err, mean_form_err];
    labels = {'Final Form Err', 'Mean Form Err'};
    bar(metrics, 'FaceColor', [0.3, 0.6, 0.8]);
    set(gca, 'XTickLabel', labels, 'XTickLabelRotation', 20);
    ylabel('Error (m)');
    title('Performance Metrics');
    grid on;
    for i = 1:length(metrics)
        text(i, metrics(i) + 0.01, sprintf('%.3f', metrics(i)), ...
             'HorizontalAlignment', 'center', 'FontSize', 10);
    end

    sgtitle('Multi-Agent Formation Simulation Results');

    % === 多格式保存 ===
    output_dir = fullfile(pwd, 'results');
    if ~exist(output_dir, 'dir'), mkdir(output_dir); end
    print(gcf, fullfile(output_dir, 'formation_results'), '-dpng', '-r300');  % 300 DPI
    fprintf('结果图片已保存到: %s\n', fullfile(output_dir, 'formation_results.png'));
end
