% 参数设置
num_agents = 5; % 智能体数量
dt = 0.01; % 时间步长
total_time = 30; % 总时间

% 初始位置和速度（在三维空间中）
positions = rand(num_agents, 3) * 10; % 随机初始位置（3D）
velocities = rand(num_agents, 3) * 2; % 随机初始速度（3D）
% 存储位置和速度的历史记录
positions_history = zeros(num_agents, 3, total_time/dt);
velocities_history = zeros(num_agents, 3, total_time/dt);

% 邻居关系（全连接）
adjacency_matrix = ones(num_agents) - eye(num_agents);

% 一致性算法参数
alpha = 1; % 位置一致性增益
beta = 0.5; % 速度一致性增益

% 可视化初始化
figure;
hold on;
colors = rand(num_agents, 3); % 随机生成 RGB 颜色值
markers = {'o', 's', 'd', '^', 'v'}; % 不同的标记类型
markerSize = 8; % 标记大小
lineWidth = 1.5; % 线宽

% 绘制轨迹线
lines = gobjects(num_agents, 1);
for i = 1:num_agents
    lines(i) = plot3(nan, nan, nan, 'Color', colors(i,:), 'LineWidth', lineWidth);
end

% 设置三维绘图的轴范围和标签
xlim([-50, 50]);
ylim([-50, 50]);
zlim([-50, 50]);
title('智能体轨迹（3D）');
xlabel('X 位置');
ylabel('Y 位置');
zlabel('Z 位置');
grid on;

% 设置新的显示角度
azimuth = 45; % 方位角（水平旋转角度）
elevation = 30; % 仰角（垂直旋转角度）
view(azimuth, elevation);




% 主循环
for t = 1:(total_time/dt)
    % 保存当前状态
    positions_history(:, :, t) = positions;
    velocities_history(:, :, t) = velocities;
    
    % 更新速度和位置
    for i = 1:num_agents
        position_diff_sum = zeros(1, 3);
        velocity_diff_sum = zeros(1, 3);
        for j = 1:num_agents
            if adjacency_matrix(i, j) == 1
                position_diff_sum = position_diff_sum + (positions(j, :) - positions(i, :));
                velocity_diff_sum = velocity_diff_sum + (velocities(j, :) - velocities(i, :));
            end
        end
        % 更新速度和位置
        velocities(i, :) = velocities(i, :) + dt * (alpha * position_diff_sum + beta * velocity_diff_sum);
        positions(i, :) = positions(i, :) + dt * velocities(i, :);
    end
    
    % 实时更新轨迹线
    for i = 1:num_agents
        set(lines(i), 'XData', [get(lines(i), 'XData'), positions(i, 1)]);
        set(lines(i), 'YData', [get(lines(i), 'YData'), positions(i, 2)]);
        set(lines(i), 'ZData', [get(lines(i), 'ZData'), positions(i, 3)]);
    end

    % % 更新视角，实时跟随第一个智能体
    % camtarget(positions(1, :));
    % campos(positions(1, :) + [1, 1, 1]); % 调整相机位置偏移
    % 
    drawnow;
end




% % 绘制位置随时间变化的图（每个维度）
% time_vector = 0:dt:(total_time-dt);
% figure;
% for dim = 1:3
%     subplot(3, 1, dim);
%     hold on;
%     for i = 1:num_agents
%         plot(time_vector, squeeze(positions_history(i, dim, :)), 'Color', colors(i,:), 'LineWidth', lineWidth);
%     end
%     title(['智能体位置（维度 ' num2str(dim) '）随时间变化']);
%     xlabel('时间');
%     ylabel(['位置（维度 ' num2str(dim) '）']);
%     legend(arrayfun(@(x) ['智能体' num2str(x)], 1:num_agents, 'UniformOutput', false));
% end

% % 绘制历史位置轨迹（3D）
% figure;
% hold on;
% for i = 1:num_agents
%     plot3(squeeze(positions_history(i, 1, :)), squeeze(positions_history(i, 2, :)), squeeze(positions_history(i, 3, :)), 'Color', colors(i,:), 'LineWidth', lineWidth);
%     plot3(positions(i, 1), positions(i, 2), positions(i, 3), markers{i}, 'Color', colors(i,:), 'MarkerSize', markerSize, 'LineWidth', lineWidth);
% end
% xlim([-50, 50]);
% ylim([-50, 50]);
% zlim([-50, 50]);
% title('智能体历史位置轨迹（3D）');
% xlabel('X 位置');
% ylabel('Y 位置');
% zlabel('Z 位置');
% grid on;
% 
% % 设置新的显示角度
% view(azimuth, elevation);

% 绘制位置随时间变化的图（每个维度）
time_vector = 0:dt:(total_time-dt);
figure;
for dim = 1:3
    subplot(3, 1, dim);
    hold on;
    for i = 1:num_agents
        plot(time_vector, squeeze(positions_history(i, dim, :)), 'Color', colors(i,:), 'LineWidth', lineWidth);
    end
    title(['智能体位置（维度 ' num2str(dim) '）随时间变化']);
    xlabel('时间');
    ylabel(['位置（维度 ' num2str(dim) '）']);
    legend(arrayfun(@(x) ['智能体' num2str(x)], 1:num_agents, 'UniformOutput', false));
end

% 绘制速度随时间变化的图（每个维度）
time_vector = 0:dt:(total_time-dt);
figure;
for dim = 1:3
    subplot(3, 1, dim);
    hold on;
    for i = 1:num_agents
        plot(time_vector, squeeze(velocities_history(i, dim, :)), 'Color', colors(i,:), 'LineWidth', lineWidth);
    end
    title(['智能体速度（维度 ' num2str(dim) '）随时间变化']);
    xlabel('时间');
    ylabel(['速度（维度 ' num2str(dim) '）']);
    legend(arrayfun(@(x) ['智能体' num2str(x)], 1:num_agents, 'UniformOutput', false));
end
