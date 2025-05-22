% % modules/plot_velocity_position_time.m
% function plot_velocity_position_time(state_hist, dt, ~)
%     [N, ~, T] = size(state_hist); t = (0:T-1) * dt;
%     colororder = lines(N);
% 
%     % Position plot with tiledlayout
%     figure('Name','Position','Units','centimeters','Position',[5,5,18,10]);
%     tl1 = tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
%     for i = 1:N
%         nexttile(1); hold on;
%         plot(t, squeeze(state_hist(:, i, 1)), 'Color', colororder(i,:), 'LineWidth', 0.8, 'DisplayName', sprintf('Agent %d', i));
% 
%         nexttile(2); hold on;
%         plot(t, squeeze(state_hist(:, i, 3)), 'Color', colororder(i,:), 'LineWidth', 0.8,'DisplayName', sprintf('Agent %d', i));
% 
%         nexttile(3); hold on;
%         plot(t, vecnorm(squeeze(state_hist(:, i, [1,3]))', 2, 2), 'Color', colororder(i,:),'LineWidth', 0.8, 'DisplayName', sprintf('Agent %d', i));
%     end
%     nexttile(1); title('X Position'); grid on; ylabel('X Position [m]'); legend('show','Location','northeastoutside');
%     nexttile(2); title('Y Position'); grid on; ylabel('Y Position [m]');legend('show','Location','northeastoutside');
%     nexttile(3); title('Position Magnitude'); grid on; xlabel('Time [s]'); ylabel('Magnitude [m]');legend('show','Location','northeastoutside');
% 
%     % Velocity plot with tiledlayout
%     figure('Name','Velocity','Units','centimeters','Position',[5,5,18,10]);
%     tl2 = tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
%     for i = 1:N
%         nexttile(1); hold on;
%         plot(t, squeeze(state_hist(:, i, 2)), 'Color', colororder(i,:),'LineWidth', 0.8, 'DisplayName', sprintf('Agent %d', i));
% 
%         nexttile(2); hold on;
%         plot(t, squeeze(state_hist(:, i, 4)), 'Color', colororder(i,:), 'LineWidth', 0.8,'DisplayName', sprintf('Agent %d', i));
% 
%         nexttile(3); hold on;
%         pplot(t, vecnorm(squeeze(state_hist(:, i, [2,4]))', 2, 2), 'Color', colororder(i,:),'LineWidth', 0.8,'DisplayName', sprintf('Agent %d', i));
%     end
%     nexttile(1); title('X Velocity'); grid on; ylabel('X Velocity [m/s]'); legend('show','Location','northeastoutside');
%     nexttile(2); title('Y Velocity'); grid on; ylabel('Y Velocity [m/s]');legend('show','Location','northeastoutside');
%     nexttile(3); title('Velocity Magnitude'); grid on; xlabel('Time [s]'); ylabel('Magnitude [m/s]');legend('show','Location','northeastoutside');
% end


% modules/plot_velocity_position_time.m -- 优化版
function plot_velocity_position_time(state_hist, dt, ~)
    % 输入：state_hist - [steps, num_agents, n_dim] 状态历史
    %      dt - 时间步长
    
    [steps, num_agents, n_dim] = size(state_hist);
    t = (0:steps-1) * dt; % 时间向量
    colororder = lines(num_agents);
    
    % Position plot with tiledlayout
    figure('Name','Position vs Time','Units','centimeters','Position',[5,5,18,10]);
    tl1 = tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
    
    % 绘制所有智能体的位置
    for i = 1:num_agents
        % X位置
        nexttile(1); hold on;
        plot(t, state_hist(:, i, 1), 'Color', colororder(i,:), ...
            'LineWidth', 0.8, 'DisplayName', sprintf('Agent %d', i));
        
        % Y位置
        nexttile(2); hold on;
        plot(t, state_hist(:, i, 3), 'Color', colororder(i,:), ...
            'LineWidth', 0.8, 'DisplayName', sprintf('Agent %d', i));
        
        % 位置大小（距离原点的距离）
        nexttile(3); hold on;
        pos_magnitude = sqrt(state_hist(:, i, 1).^2 + state_hist(:, i, 3).^2);
        plot(t, pos_magnitude, 'Color', colororder(i,:), ...
            'LineWidth', 0.8, 'DisplayName', sprintf('Agent %d', i));
    end
    
    % 设置位置图的标题和标签
    nexttile(1); 
    title('X Position vs Time'); 
    grid on; 
    ylabel('X Position [m]'); 
    legend('show', 'Location', 'best');
    
    nexttile(2); 
    title('Y Position vs Time'); 
    grid on; 
    ylabel('Y Position [m]');
    legend('show', 'Location', 'best');
    
    nexttile(3); 
    title('Position Magnitude vs Time'); 
    grid on; 
    xlabel('Time [s]'); 
    ylabel('Distance from Origin [m]');
    legend('show', 'Location', 'best');
    
    % Velocity plot with tiledlayout
    figure('Name','Velocity vs Time','Units','centimeters','Position',[25,5,18,10]);
    tl2 = tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
    
    % 绘制所有智能体的速度
    for i = 1:num_agents
        % X速度
        nexttile(1); hold on;
        plot(t, state_hist(:, i, 2), 'Color', colororder(i,:), ...
            'LineWidth', 0.8, 'DisplayName', sprintf('Agent %d', i));
        
        % Y速度
        nexttile(2); hold on;
        plot(t, state_hist(:, i, 4), 'Color', colororder(i,:), ...
            'LineWidth', 0.8, 'DisplayName', sprintf('Agent %d', i));
        
        % 速度大小
        nexttile(3); hold on;
        vel_magnitude = sqrt(state_hist(:, i, 2).^2 + state_hist(:, i, 4).^2);
        plot(t, vel_magnitude, 'Color', colororder(i,:), ...
            'LineWidth', 0.8, 'DisplayName', sprintf('Agent %d', i));
    end
    
    % 设置速度图的标题和标签
    nexttile(1); 
    title('X Velocity vs Time'); 
    grid on; 
    ylabel('X Velocity [m/s]'); 
    legend('show', 'Location', 'best');
    
    nexttile(2); 
    title('Y Velocity vs Time'); 
    grid on; 
    ylabel('Y Velocity [m/s]');
    legend('show', 'Location', 'best');
    
    nexttile(3); 
    title('Velocity Magnitude vs Time'); 
    grid on; 
    xlabel('Time [s]'); 
    ylabel('Speed [m/s]');
    legend('show', 'Location', 'best');
end
