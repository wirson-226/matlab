% 五个智能体在圆周上形成正五边形的固定编队

% 设置智能体数量和圆周半径
n = 5;
r = 1;

% 初始化智能体随机位置和速度
positions = 2 * r * (rand(2, n) - 0.5);  % 初始位置随机分布在[-r, r]范围内
velocities = zeros(2, n);  % 初始速度为零

% 初始控制增益矩阵
K1 = kron(eye(2), [-2, -1.2]);  % 使用 Kronecker 积计算增益矩阵
K2 = 0.5 * eye(2);  % 邻居位置控制增益
Kv = 0.5 * eye(2);  % 速度控制增益
Ki = 0.1 * eye(2);  % 积分控制增益

% 自适应增益的学习率
alpha = 0.01;

% 仿真参数
dt = 0.1;  % 时间步长
simulation_time = 20;  % 仿真总时间
num_steps = simulation_time / dt;

% 计算期望位置
theta = linspace(0, 2*pi, n+1);
theta = theta(1:end-1);
desired_positions = [r * cos(theta); r * sin(theta)];

% 初始化轨迹和误差
trajectories = cell(1, n);
position_errors = zeros(n, num_steps);
integral_errors = zeros(2, n);  % 积分误差初始化为零

for i = 1:n
    trajectories{i} = zeros(2, num_steps);
end

% 主循环进行仿真
for t = 1:num_steps
    % 记录当前智能体位置和误差
    for i = 1:n
        trajectories{i}(:, t) = positions(:, i);
        position_errors(i, t) = norm(desired_positions(:, i) - positions(:, i));
    end
    
    % 计算每个智能体的控制力
    control_forces = zeros(2, n);
    
    for i = 1:n
        % 计算位置和速度误差
        position_error = desired_positions(:, i) - positions(:, i);
        velocity_error = - velocities(:, i);  % 速度误差
        
        % 更新积分误差
        integral_errors(:, i) = integral_errors(:, i) + position_error * dt;
        
        % 自适应调整控制增益
        K1 = K1 - alpha * (position_error * position_error');
        
        % 位置和速度控制力
        u1 = K1 * [position_error; velocity_error];
        ui = Ki * integral_errors(:, i);
        
        % 计算第二个控制输入部分
        u2 = zeros(2, 1);
        for j = 1:n
            if i ~= j
                position_error_neighbor = (positions(:, j) - positions(:, i)) - (desired_positions(:, j) - desired_positions(:, i));
                u2 = u2 + K2 * position_error_neighbor;
            end
        end
        
        % 总控制力
        control_force = u1 + u2 + ui;
        
        % 更新速度和位置
        velocities(:, i) = velocities(:, i) + control_force * dt;
        positions(:, i) = positions(:, i) + velocities(:, i) * dt;
    end
    
    % 绘制当前时刻的智能体位置和轨迹
    figure(1);
    clf; % 清除当前图形窗口
    hold on;
    
    % 绘制轨迹线
    for i = 1:n
        plot(trajectories{i}(1, 1:t), trajectories{i}(2, 1:t), '--', 'Color', [0 0 1], 'LineWidth', 1.5); % 用虚线绘制轨迹
    end
    
    % 绘制当前智能体位置
    plot(positions(1, :), positions(2, :), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    plot(desired_positions(1, :), desired_positions(2, :), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');  % 绘制期望位置
    
    % 按顺序用虚线连接智能体
    for i = 1:n
        next_i = mod(i, n) + 1;
        plot([positions(1, i), positions(1, next_i)], [positions(2, i), positions(2, next_i)], 'b--', 'LineWidth', 1.5);
    end
    
    % 绘制期望位置的圆弧连线
    theta_dense = linspace(0, 2*pi, 100);
    plot(r * cos(theta_dense), r * sin(theta_dense), 'r--', 'LineWidth', 1.5);
    
    hold off;
    axis equal;
    title(sprintf('Time: %.1f', t*dt));
    xlabel('X Position');
    ylabel('Y Position');
    xlim([-2*r 2*r]);
    ylim([-2*r 2*r]);
    grid on;
    legend('Trajectory', 'Current Position', 'Desired Position', 'Location', 'Best');
    pause(0.1);
end

% 绘制位置误差图像
figure(2);
hold on;
for i = 1:n
    plot((1:num_steps) * dt, position_errors(i, :), 'LineWidth', 1.5);
end
hold off;
xlabel('Time (s)');
ylabel('Position Error');
title('Position Error of Each Agent Over Time');
legend(arrayfun(@(i) sprintf('Agent %d', i), 1:n, 'UniformOutput', false), 'Location', 'Best');
grid on;
