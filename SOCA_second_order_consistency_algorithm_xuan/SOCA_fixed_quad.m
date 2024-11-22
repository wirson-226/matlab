% 参数设置
num_agents = 5; % 智能体数量（包括中心和四个顶点）
dt = 0.1; % 时间步长
total_time = 20; % 总时间
side_length = 3; % 正方形边长
desired_speed = 1; % 初始期望速度
consensus_gain = 0.1; % 一致性增益
velocity_decay_rate = 0.01; % 速度衰减率

% 初始位置和速度的随机范围
position_range = 10; % 位置范围 [-position_range/2, position_range/2]
velocity_range = 2; % 速度范围 [-velocity_range/2, velocity_range/2]

% 初始位置和速度（三维空间）
positions = position_range * (rand(num_agents, 3) - 0.5); % 随机初始位置
velocities = velocity_range * (rand(num_agents, 3) - 0.5); % 随机初始速度

% 存储位置的历史记录
positions_history = zeros(num_agents, 3, total_time/dt);

% 创建图形窗口
figure;
hold on;
grid on;
xlabel('X 位置');
ylabel('Y 位置');
zlabel('Z 位置');
title('多智能体正方形平移编队包括中心示例（随机初始位置和速度，带一致性增益和连线）');
view(3); % 设置为三维视角

% 主循环
for t = 1:(total_time/dt)
    % 保存当前状态
    positions_history(:, :, t) = positions;
    
 
    % 计算每个智能体的目标位置（固定正方形顶点位置）
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
        
        % 逐步减小期望速度
        desired_speed = max(0, desired_speed - velocity_decay_rate * dt);
        
        velocities_norm = norm(velocities(i, :));
        if velocities_norm > 0
            velocities(i, :) = velocities(i, :) / velocities_norm * desired_speed;
        end
        
        positions(i, :) = positions(i, :) + dt * velocities(i, :);
    end
        % 绘制当前位置和连线
    for i = 1:num_agents
        plot3(positions(i, 1), positions(i, 2), positions(i, 3), 'o', 'MarkerSize', 8, 'LineWidth', 1.5);
    end
    
    % 绘制连线
    for i = 1:num_agents
        for j = i+1:num_agents
            plot3([positions(i, 1), positions(j, 1)], ...
                  [positions(i, 2), positions(j, 2)], ...
                  [positions(i, 3), positions(j, 3)], '-', 'Color', [0.5 0.5 0.5]);
        end
    end
    
    drawnow; % 实时更新绘图
   
end

