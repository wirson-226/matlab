% 五个智能体在圆周上形成正五边形的固定编队

% 设置智能体数量和圆周半径
n = 5;
r = 1;
theta = linspace(0, 2*pi, n+1);
theta = theta(1:end-1);
range = 3 * r;


% 初始化智能体位置和速度

% ######位置########
% 目标点附近
% positions = [r * cos(theta); r * sin(theta)] + 0.1 * randn(2, n);  % 初始位置略微偏离
% 初始小偏差
% positions = 0.5 * range * (rand(2, n) - 0.5);  % 初始位置随机分布在0.25 * [-range,range]范围内
% 初始中偏差
% positions = range * (rand(2, n) - 0.5);  % 初始位置随机分布在0.5 * [-range,range]范围内
% 初始大偏差
positions = 2 * range * (rand(2, n) - 0.5);  % 初始位置随机分布在[-range,range]范围内

% ######速度########
velocities = zeros(2, n);  % 初始速度为零

% 控制增益矩阵 速度响应快略大些，位置维持队形
% 很快
% K1 = 9 * eye(2);  % 位置控制增益
% K2 = 9 * eye(2);  % 邻居位置控制增益
% Kv = 10 * eye(2);  % 速度控制增益  

% 略快
K1 = 3 * eye(2);  % 位置控制增益
K2 = 3 * eye(2);  % 邻居位置控制增益
Kv = 4 * eye(2);  % 速度控制增益  

% 慢速演示
% K1 = 0.7 * eye(2);  % 位置控制增益
% K2 = 0.7 * eye(2);  % 邻居位置控制增益
% Kv = 2 * eye(2);  % 速度控制增益  

% 设置智能体显示名称，颜色
% names = ['智能体1','智能体2','智能体3','智能体4','智能体5'];
colors = [0 0.4470 0.741   
          0.8500 0.3250 0.0980
          0.9290 0.6940 0.1250
          0.4940 0.1840 0.5560
          0.4660 0.6740 0.1880
        ];

% 仿真参数
dt = 0.1;  % 时间步长
simulation_time = 10;  % 仿真总时间
num_steps = simulation_time / dt; % 仿真步数

% 存储位置的历史记录
positions_history = zeros(2, n, num_steps);
velocities_history = zeros(2, n, num_steps);


% ######计算期望位置#######
desired_positions = [r * cos(theta); r * sin(theta)];% 根据需要提供编队形式

% ######计算期望速度#######
desired_velocities = zeros(2, n);% 根据需要提供编队形式

% 初始化轨迹
trajectories = cell(1, n);
for i = 1:n
    trajectories{i} = zeros(2, num_steps);
end

% 主循环进行仿真
for t = 1:num_steps

    % 保存当前状态
    positions_history(:, :, t) = positions;
    velocities_history(:, :, t) = velocities;

    % 记录当前智能体位置
    for i = 1:n
        trajectories{i}(:, t) = positions(:, i);
    end

    % 计算每个智能体的控制力
    control_forces = zeros(2, n);
    
    for i = 1:n
        % 计算第一个控制输入部分
        position_error = desired_positions(:, i) - positions(:, i); % 位置误差
        velocity_error = desired_velocities(:, i) - velocities(:, i);  % 速度误差
        
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
        
        % 更新速度和位置
        velocities(:, i) = velocities(:, i) + control_force * dt;
        positions(:, i) = positions(:, i) + velocities(:, i) * dt;
    end

    % #####图形示例#####
    % 添加实时轨迹
    % 绘制当前时刻的智能体位置和轨迹
    figure(1);
    clf; % 清除当前图形窗口
    hold on;
    
    % 绘制轨迹线
    for i = 1:n
        % plot(trajectories{i}(1, 1:t), trajectories{i}(2, 1:t), '-', 'Color', [0.4940 0.1840 0.5560], 'LineWidth', 1.5); % 用虚线绘制轨迹
        plot(trajectories{i}(1, 1:t), trajectories{i}(2, 1:t), '-', 'Color', colors(i,:), 'LineWidth', 1); % 多组颜色
    end
    
    % 绘制位置
    for i = 1:n
        plot(positions(1, i), positions(2, i), '.', 'Color', colors(i,:), 'MarkerSize', 50); % 绘制当前智能体位置
        plot(desired_positions(1, i), desired_positions(2, i), 'o','Color', colors(i,:), 'MarkerSize', 15);  % 绘制期望位置
    end
    
    
    % 按顺序用虚线连接智能体
    for i = 1:n
        next_i = mod(i, n) + 1;
        plot([positions(1, i), positions(1, next_i)], [positions(2, i), positions(2, next_i)], '--', 'Color', [0,0,0], 'LineWidth', 1.5);
    end
    
    % 绘制起始点
    for i = 1:n
        plot(trajectories{i}(1, 1), trajectories{i}(2, 1), 'pentagram', 'Color', colors(i,:), 'MarkerSize', 10); % 起点
    end

    % 绘制期望位置的圆弧连线
    theta_dense = linspace(0, 2*pi, 100);
    plot(r * cos(theta_dense), r * sin(theta_dense), 'r--', 'LineWidth', 0.5);
    plot(0, 0, '.', 'Color', [1,0,0], 'MarkerSize', 10); % 中心原点 options:"o""+""*"	"."	"x"	"_"	"|"	"square""diamond""^""v"	">"	"<""pentagram"	"hexagram""none"

    
    % 添加图注，设置网格
    hold off;
    axis equal;
    title(sprintf('Time: %.1f', t*dt));
    xlabel('X Position');
    ylabel('Y Position');
    xlim([-range range]);
    ylim([-range range]);
    grid on;
    legend('智能体1','智能体2','智能体3','智能体4','智能体5','当前位置','期望位置','','','','','','','','','','','','','','起始位置','Location', 'Best');
    pause(0.1);

    % #####添加仿真终止判定#####

    % % 如果速度足够小，则终止仿真
    % if norm(velocities) < 0.001
    %     break;
    % end

    % 如果位移足够小，则终止仿真
    if norm(position_error) < 0.0001
        break;
    end

end



% 绘制位置随时间变化的图（每个维度）
time_vector = 0:dt:(simulation_time-dt);
figure;
for dim = 1:2
    subplot(2, 1, dim);
    hold on;
    for i = 1:n
        plot(time_vector, squeeze(positions_history(dim, i, :)), 'Color', colors(i,:), 'LineWidth', 1.5);
    end
    title(['智能体位置（维度方向按序x,y ' num2str(dim) '）随时间变化']);
    xlabel('时间');
    ylabel(['位置（维度方向按序x,y' num2str(dim) '）']);
    legend(arrayfun(@(x) ['智能体' num2str(x)], 1:n, 'UniformOutput', false));
end

% 绘制速度随时间变化的图（每个维度）
time_vector = 0:dt:(simulation_time-dt);
figure;
for dim = 1:2
    subplot(2, 1, dim);
    hold on;
    for i = 1:n
        plot(time_vector, squeeze(velocities_history(dim, i, :)), 'Color', colors(i,:), 'LineWidth', 1.5);
    end
    title(['智能体速度（维度方向按序x,y ' num2str(dim) '）随时间变化']);
    xlabel('时间');
    ylabel(['速度（维度方向按序x,y' num2str(dim) '）']);
    legend(arrayfun(@(x) ['智能体' num2str(x)], 1:n, 'UniformOutput', false));
end


% 初始化：
% 
% 设置智能体数量 n 和圆周半径 r。
% 初始化智能体的位置 positions 和速度 velocities。
% 设置控制增益矩阵 K1、K2 和 Kv。
% 期望位置：
% 
% 计算期望位置 desired_positions，使智能体以设计的编队形式进行运动
% 主仿真循环：
% 
% 在每个仿真时间步内，计算每个智能体的控制力。
% 位置控制力 u1 由位置误差计算得到。
% 速度控制力 uv 由速度误差计算得到。
% 邻居位置控制力 u2 由当前智能体与邻居智能体之间的位置误差计算得到。
% 总控制力 control_force 是三部分控制力的和。
% 更新每个智能体的速度和位置。
% 绘制当前时刻的智能体位置和期望位置。
% 一致性不一定分布式，可以集中式拿到信息在利用邻居的相关状态进行控制效果提升

%####注意#####
%后续调整配色显示优化美观
