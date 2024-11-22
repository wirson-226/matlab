% 五个智能体在圆周上形成正五边形的固定编队

% 设置智能体数量和圆周半径
n = 5;
r = 1;
theta = linspace(0, 2*pi, n+1);
theta = theta(1:end-1);

% 初始化智能体位置和速度
positions = [r * cos(theta); r * sin(theta)] + 0.1 * randn(2, n);  % 初始位置略微偏离
velocities = zeros(2, n);  % 初始速度为零

% 控制增益矩阵
K1 = 1 * eye(2);  % 第一部分控制输入的增益
K2 = 0.5 * eye(2);  % 第二部分控制输入的增益
Kv = 0.5 * eye(2);  % 速度相关的控制增益

% 仿真参数
dt = 0.1;  % 时间步长
simulation_time = 20;  % 仿真总时间

% 计算期望位置
desired_positions = [r * cos(theta); r * sin(theta)];

% 初始化观测器
A = kron(eye(2), [1, dt; 0, 1]);
B = kron(eye(2), [dt^2/2; dt]);
observers = cell(1, n);
for i = 1:n
    observers{i} = [positions(:, i); velocities(:, i)];
end

% 主循环进行仿真
for t = 0:dt:simulation_time
    % 计算每个智能体的控制力
    control_forces = zeros(2, n);
    
    for i = 1:n
        % 使用观测器估计状态
        estimated_state = observers{i};
        estimated_position = estimated_state(1:2);
        estimated_velocity = estimated_state(3:4);
        
        % 计算第一个控制输入部分
        position_error = desired_positions(:, i) - estimated_position;
        velocity_error = - estimated_velocity;  % 速度误差
        
        % 位置控制力
        u1 = K1 * position_error;
        
        % 速度控制力
        uv = Kv * velocity_error;
        
        % 计算第二个控制输入部分
        u2 = zeros(2, 1);
        for j = 1:n
            if i ~= j
                position_error_neighbor = (positions(:, j) - positions(:, i)) - (desired_positions(:, j) - desired_positions(:, i));
                u2 = u2 + K2 * position_error_neighbor;
            end
        end
        
        % 总控制力
        control_force = u1 + u2 + uv;
        
        % 更新观测器状态
        observers{i} = A * observers{i} + B * control_force;
        
        % 更新实际速度和位置
        velocities(:, i) = velocities(:, i) + control_force * dt;
        positions(:, i) = positions(:, i) + velocities(:, i) * dt;
    end
    
    % 绘制当前时刻的智能体位置
    figure(1);
    plot(positions(1, :), positions(2, :), 'bo', 'MarkerSize', 10);
    hold on;
    plot(desired_positions(1, :), desired_positions(2, :), 'ro', 'MarkerSize', 10);  % 绘制期望位置
    hold off;
    axis equal;
    title(sprintf('Time: %.1f', t));
    xlim([-1.5*r 1.5*r]);
    ylim([-1.5*r 1.5*r]);
    pause(0.1);
end


% 观测器的状态更新实际上并未对智能体的控制产生直接影响。要让观测器真正参与到控制效果中，我们需要使用观测器的状态估计来计算控制输入，而不是直接使用实际状态。

% 我们可以修改代码，使观测器参与到控制算法中。具体来说，我们将使用观测器的状态估计值来替代实际的速度和位置，从而计算控制力。