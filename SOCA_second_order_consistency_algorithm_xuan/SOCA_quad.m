% 参数设置
num_agents = 5; % 智能体数量（包括中心和四个顶点）
dt = 0.1; % 时间步长
total_time = 100; % 总时间
side_length = 3; % 正方形边长
desired_speed = 0.5; % 期望速度
consensus_gain = 0.3; % 一致性增益

% 初始位置和速度的随机范围
position_range = 10; % 位置范围 [-position_range/2, position_range/2]
velocity_range = 2; % 速度范围 [-velocity_range/2, velocity_range/2]

% 初始位置和速度（三维空间）
positions = position_range * (rand(num_agents, 3) - 0.5); % 随机初始位置
velocities = velocity_range * (rand(num_agents, 3) - 0.5); % 随机初始速度

% 存储位置的历史记录
positions_history = zeros(num_agents, 3, total_time/dt);
velocities_history = zeros(num_agents, 3, total_time/dt);
% 可视化初始化
figure;
hold on;
% colors = rand(num_agents, 3); % 随机生成 RGB 颜色值
colors = [
    0 0 1
    0 0 1
    0 0 1
    0 0 1
    1 0 0
        ]; % 随机生成 RGB 颜色值
markers = {'o', 's', 'd', '^', 'v'}; % 不同的标记类型
markerSize = 8; % 标记大小
lineWidth = 1.5; % 线宽

% 绘制轨迹线
lines = gobjects(num_agents, 1);
for i = 1:num_agents
    lines(i) = plot3(nan, nan, nan, 'Color', colors(i,:), 'LineWidth', lineWidth);
end

% 设置三维绘图的轴范围和标签

title('智能体轨迹（3D）');
xlabel('X 位置');
ylabel('Y 位置');
zlabel('Z 位置');
grid on;

% 初始设置显示角度
azimuth = 0; % 方位角（水平旋转角度）
elevation = 90; % 仰角（垂直旋转角度）
view(azimuth, elevation);



% 主循环
for t = 1:(total_time/dt)
    % 保存当前状态
    positions_history(:, :, t) = positions;
    velocities_history(:, :, t) = velocities;
    % 计算每个智能体的目标位置（正方形顶点位置）
    target_positions = [-side_length/2, -side_length/2, 0; % 左下角
                        side_length/2, -side_length/2, 0;  % 右下角
                        side_length/2, side_length/2, 0;   % 右上角
                        -side_length/2, side_length/2, 0;  % 左上角
                        0, 0, 0]; % 中心位置
    
    % 分布式控制更新
    for i = 1:num_agents
        % 计算邻居平均速度
        avg_velocity = zeros(1, 3);
        for j = 1:num_agents
            if i ~= j
                avg_velocity = avg_velocity + velocities(j, :);
            end
        end
        if num_agents > 1
            avg_velocity = avg_velocity / (num_agents - 1);
        end
        
        % 更新速度和位置（包括一致性增益）
        velocities(i, :) = velocities(i, :) + dt * consensus_gain * (avg_velocity - velocities(i, :));
        velocities_norm = norm(velocities(i, :));
        if velocities_norm > 0
            velocities(i, :) = velocities(i, :) / velocities_norm * desired_speed;
        end
        
        positions(i, :) = positions(i, :) + dt * velocities(i, :);
    end
        % 实时更新轨迹线
    for i = 1:num_agents
        set(lines(i), 'XData', [get(lines(i), 'XData'), positions(i, 1)]);
        set(lines(i), 'YData', [get(lines(i), 'YData'), positions(i, 2)]);
        set(lines(i), 'ZData', [get(lines(i), 'ZData'), positions(i, 3)]);
    end

    % % 更新视角，实时跟随中心智能体
    % camtarget(center_position);
    % campos(center_position + [3, 3, 3]); % 调整相机位置偏移
    % 
    drawnow;
end




% 绘制位置随时间变化的图（每个维度）
time_vector = 0:dt:(total_time-dt);
figure;
for dim = 1:3
    subplot(3, 1, dim);
    hold on;
    for i = 1:num_agents
        plot(time_vector, squeeze(positions_history(i, dim, :)), 'Color', colors(i,:), 'LineWidth', lineWidth);
    end
    title(['智能体位置（维度方向按序x,y,Z ' num2str(dim) '）随时间变化']);
    xlabel('时间');
    ylabel(['位置（维度方向按序x,y,Z ' num2str(dim) '）']);
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
    title(['智能体速度（维度方向按序x,y,Z ' num2str(dim) '）随时间变化']);
    xlabel('时间');
    ylabel(['速度（维度方向按序x,y,Z ' num2str(dim) '）']);
    legend(arrayfun(@(x) ['智能体' num2str(x)], 1:num_agents, 'UniformOutput', false));
end




% 任务一 二阶系统  固定编队的实现，时变就是编队队形的偏差值介入 

% 任务二 六自由度  旋翼智能体控制框架  kumar 课程实现，一致性算法替换位置输入路径规划