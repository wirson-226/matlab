% 参数设置
num_agents = 5; % 智能体数量（包括1个中心智能体和4个运动智能体）
dt = 0.01; % 时间步长
total_time = 20; % 总时间
radius = 5; % 圆周运动半径
angular_velocity = pi/4; % 角速度
vertical_velocity = 0.5; % 初始垂直方向速度

% 初始位置和速度（在三维空间中）
center_position = [5, 5, 5]; % 中心智能体的位置（固定）
angles = linspace(0, 2*pi, num_agents + 1); % 初始角度
angles(end) = []; % 去除重复角度
positions = zeros(num_agents, 3); % 初始化位置数组
velocities = zeros(num_agents, 3); % 初始化速度数组

% 初始化运动智能体的位置和速度（围绕中心智能体）
for i = 2:num_agents
    % 初始位置
    initial_angle = rand() * 2 * pi;
    positions(i, 1:2) = center_position(1:2) + radius * [cos(initial_angle), sin(initial_angle)];
    velocities(i, 1:2) = [-sin(initial_angle), cos(initial_angle)] * angular_velocity * radius;
    positions(i, 3) = rand() * 10; % 初始垂直位置
    velocities(i, 3) = vertical_velocity; % 初始垂直速度
end
positions(1, :) = center_position; % 中心智能体位置

% 存储位置和速度的历史记录
positions_history = zeros(num_agents, 3, total_time/dt);
velocities_history = zeros(num_agents, 3, total_time/dt);

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

% 初始设置显示角度
azimuth = 45; % 方位角（水平旋转角度）
elevation = 30; % 仰角（垂直旋转角度）
view(azimuth, elevation);

% 主循环
for t = 1:(total_time/dt)
    % 保存当前状态
    positions_history(:, :, t) = positions;
    velocities_history(:, :, t) = velocities;
    
    % 分布式更新智能体的位置和速度
    for i = 2:num_agents
        % 更新水平方向位置和速度（围绕中心运动）
        angle = angular_velocity * dt * t;
        positions(i, 1:2) = center_position(1:2) + radius * [cos(angle + angles(i)), sin(angle + angles(i))];
        velocities(i, 1:2) = [-sin(angle + angles(i)), cos(angle + angles(i))] * angular_velocity * radius;
        
        % 更新垂直方向位置和速度
        positions(i, 3) = positions(i, 3) + dt * velocities(i, 3);
    end
    
    % 实时更新轨迹线
    for i = 1:num_agents
        set(lines(i), 'XData', [get(lines(i), 'XData'), positions(i, 1)]);
        set(lines(i), 'YData', [get(lines(i), 'YData'), positions(i, 2)]);
        set(lines(i), 'ZData', [get(lines(i), 'ZData'), positions(i, 3)]);
    end
    
    drawnow;
end

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
% % 设置历史轨迹的显示角度
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
