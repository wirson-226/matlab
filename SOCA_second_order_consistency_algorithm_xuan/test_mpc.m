% 五个智能体在圆周上形成正五边形的固定编队
% 使用模型预测控制（MPC）

% 设置智能体数量和圆周半径
n = 5;
r = 1;

% 初始化智能体随机位置和速度
positions = 2 * r * (rand(2, n) - 0.5);  % 初始位置随机分布在[-r, r]范围内
velocities = zeros(2, n);  % 初始速度为零

% MPC 参数
prediction_horizon = 10;  % 预测时域长度
control_horizon = 3;  % 控制时域长度
Ts = 0.1;  % 采样时间

% 仿真参数
simulation_time = 20;  % 仿真总时间
num_steps = simulation_time / Ts;

% 计算期望位置
theta = linspace(0, 2*pi, n+1);
theta = theta(1:end-1);
desired_positions = [r * cos(theta); r * sin(theta)];

% 初始化轨迹和误差
trajectories = cell(1, n);
position_errors = zeros(n, num_steps);

for i = 1:n
    trajectories{i} = zeros(2, num_steps);
end

% 创建 MPC 控制器
mpcobj = cell(1, n);
for i = 1:n
    % 定义系统模型
    plant = ss([0 1; 0 0], [0; 1], eye(2), zeros(2, 1), Ts);
    
    % 创建 MPC 控制器
    mpcobj{i} = mpc(plant, prediction_horizon, control_horizon, Ts);
    
    % 设置权重
    mpcobj{i}.Weights.ManipulatedVariables = 0.1;  % 单一值表示所有操纵变量的权重
    mpcobj{i}.Weights.ManipulatedVariablesRate = 0.1;
    mpcobj{i}.Weights.OutputVariables = [1 1];  % 修正为矩阵表示所有输出变量的权重
    
    % 设置输入约束
    mpcobj{i}.MV(1).Min = -Inf;
    mpcobj{i}.MV(1).Max = Inf;
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
        % 获取当前状态
        x = [positions(:, i); velocities(:, i)];
        
        % 获取期望输出
        ref = desired_positions(:, i);
        
        % 计算 MPC 控制输入
        u = mpcmove(mpcobj{i}, x, ref);
        
        % 更新速度和位置
        velocities(:, i) = velocities(:, i) + u * Ts;
        positions(:, i) = positions(:, i) + velocities(:, i) * Ts;
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
    title(sprintf('Time: %.1f', t*Ts));
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
    plot((1:num_steps) * Ts, position_errors(i, :), 'LineWidth', 1.5);
end
hold off;
xlabel('Time (s)');
ylabel('Position Error');
title('Position Error of Each Agent Over Time');
legend(arrayfun(@(i) sprintf('Agent %d', i), 1:n, 'UniformOutput', false), 'Location', 'Best');
grid on;


% 可参考：https://blog.csdn.net/weixin_46300916/article/details/137441145