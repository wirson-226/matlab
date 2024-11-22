% 设置智能体数量和圆周半径
n = 5;
r = 1;
theta = linspace(0, 2*pi, n+1);
theta = theta(1:end-1);
range = 3 * r;

% 初始化智能体位置和速度（三维）
positions = 2 * range * (rand(3, n) - 0.5);  % 初始位置随机分布在[-range,range]范围内
velocities = zeros(3, n);  % 初始速度为零
acceleration = zeros(3, n); % 加速度为零

% 控制增益矩阵（三维）
Ka = 0 * eye(3); % 加速度控制增益
K1 = 10 * eye(3);  % 位置控制增益
Kv1 = 8 * eye(3); % 速度控制增益
Kv2 = 1 * eye(3); % 邻居速度控制增益
K2 = 3 * eye(3);  % 邻居位置控制增益

% 设置智能体显示名称，颜色
colors = [0 0.4470 0.741   
          0.8500 0.3250 0.0980
          0.9290 0.6940 0.1250
          0.4940 0.1840 0.5560
          0.4660 0.6740 0.1880];

% 仿真参数
dt = 0.1;  % 时间步长
simulation_time = 20;  % 仿真总时间
num_steps = simulation_time / dt; % 仿真步数

% 存储位置的历史记录
positions_history = zeros(3, n, num_steps);
velocities_history = zeros(3, n, num_steps);
positions_des_history = zeros(3, n, num_steps);

% 计算期望位置和速度（三维）
desired_positions_init = [r * cos(theta); r * sin(theta); zeros(1, n)]; % 初始期望位置
desired_velocities_init = 0.2 * ones(3, n); % 初始期望速度

% 初始化轨迹
trajectories = cell(1, n);
for i = 1:n
    trajectories{i} = zeros(3, num_steps);
end

position_errors = zeros(n, num_steps);

% 主循环进行仿真
for t = 1:num_steps

    % 实时更新计算期望状态：位置与速度
    desired_velocities = desired_velocities_init + acceleration * dt * t;
    desired_positions = desired_positions_init + desired_velocities * dt * t;

    % 保存当前状态
    positions_history(:, :, t) = positions;
    velocities_history(:, :, t) = velocities;
    positions_des_history(:, :, t) = desired_positions;

    % 记录当前智能体位置
    for i = 1:n
        trajectories{i}(:, t) = positions(:, i);
        position_errors(i, t) = norm(desired_positions(:, i) - positions(:, i));
    end

    % 计算每个智能体的控制力
    for i = 1:n
        % 计算控制输入部分
        position_error = desired_positions(:, i) - positions(:, i);    % 位置误差
        velocity_error = desired_velocities(:, i) - velocities(:, i);  % 速度误差

        % 位置控制力
        u1 = K1 * position_error;

        % 速度控制力
        uv = Kv1 * velocity_error;

        % 邻居控制力
        u2 = zeros(3, 1);
        for j = 1:n
            if i ~= j
                position_error_neighbor = (positions(:, j) - positions(:, i)) - (desired_positions(:, j) - desired_positions(:, i));
                velocity_error_neighbor = (velocities(:, j) - velocities(:, i)) - (desired_velocities(:, j) - desired_velocities(:, i));
                u2 = u2 + K2 * position_error_neighbor + Kv2 * velocity_error_neighbor;
            end
        end

        % 加速度控制力
        ua = Ka * acceleration(:, i);

        % 总控制力
        control_force = u1 + u2 + uv + ua;

        % 更新速度和位置
        velocities(:, i) = velocities(:, i) + control_force * dt;
        positions(:, i) = positions(:, i) + velocities(:, i) * dt;
    end

    % 绘图部分
    figure(1);
    clf; % 清除当前图形窗口
    hold on;
    
    % 绘制轨迹线
    for i = 1:n
        plot3(trajectories{i}(1, 1:t), trajectories{i}(2, 1:t), trajectories{i}(3, 1:t), '-', 'Color', colors(i,:), 'LineWidth', 1);
    end
    
    % 绘制当前与期望位置
    for i = 1:n
        plot3(positions(1, i), positions(2, i), positions(3, i), '.', 'Color', colors(i,:), 'MarkerSize', 50);
        plot3(desired_positions(1, i), desired_positions(2, i), desired_positions(3, i), 'o', 'Color', colors(i,:), 'MarkerSize', 15);
    end
    
    % 按顺序用虚线连接智能体
    for i = 1:n
        next_i = mod(i, n) + 1;
        plot3([positions(1, i), positions(1, next_i)], [positions(2, i), positions(2, next_i)], [positions(3, i), positions(3, next_i)], '--', 'Color', [0,0,0], 'LineWidth', 1.5);
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
    zlabel('Z Position');
    grid on;
    legend('智能体1','智能体2','智能体3','智能体4','智能体5','当前位置','期望位置','','','','','','','','','','','','','','起始位置','Location', 'Best');
    pause(0.1);

    % #####添加仿真终止判定#####

    % % 如果速度足够小，则终止仿真
    % if norm(velocities) < 0.001
    %     break;
    % end

    % 如果位移足够小，则终止仿真
    % 终止条件
    if norm(position_error) < 0.001
        break;
    end

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

% 显示稳态误差
error_half_average = mean(position_errors(:, num_steps*0.5));
disp('error_half_average =');
disp(error_half_average);


% 显示最终位置偏差平均值
% error_final_average = 0.2 * (position_errors(1,num_steps) + position_errors(2,num_steps) + position_errors(3,num_steps) + position_errors(4,num_steps));
% error_final_average = 0.2 * sum(position_errors(1:5,num_steps),1);
error_final_average = mean(position_errors(:, num_steps));
disp('error_final_average =');
disp(error_final_average);
fprintf('Final average position error: %.4f\n', error_final_average);
% 显示后半程稳态误差收敛效果
error_comparision_average = 100 * (error_half_average - error_final_average)/error_half_average;
fprintf('error_comparision_average = %4.2f%%\n',error_comparision_average);
% disp(error_comparision_average);


% 绘制位置随时间变化的图（每个维度）
time_vector = 0:dt:(simulation_time-dt);
figure;
for dim = 1:3
    subplot(3, 1, dim);
    hold on;
    for i = 1:n
        plot(time_vector, squeeze(positions_history(dim, i, :)), 'Color', colors(i,:), 'LineWidth', 1.5);
    end
    % 添加期望位置的plot
    for i = 1:n
        plot(time_vector, squeeze(positions_des_history(dim, i, :)), '--', 'Color', colors(i,:), 'LineWidth', 1.5);
    end
    
    title(['智能体位置（维度方向按序x,y ' num2str(dim) '）随时间变化']);
    xlabel('时间');
    ylabel(['位置（维度方向按序x,y' num2str(dim) '）']);
    % legend(arrayfun(@(x) ['智能体' num2str(x)], 1:n, 'UniformOutput', false));
    legend([arrayfun(@(x) ['智能体' num2str(x)], 1:n, 'UniformOutput', false), '期望位置'], 'Location', 'Best');
end

% 绘制速度随时间变化的图（每个维度）
figure;
for dim = 1:3
    subplot(3, 1, dim);
    hold on;
    for i = 1:n
        plot(time_vector, squeeze(velocities_history(dim, i, :)), 'Color', colors(i,:), 'LineWidth', 1.5);
    end
    title(['智能体速度（维度方向按序x,y ' num2str(dim) '）随时间变化']);
    xlabel('时间');
    ylabel(['速度（维度方向按序x,y' num2str(dim) '）']);
    legend(arrayfun(@(x) ['智能体' num2str(x)], 1:n, 'UniformOutput', false));
end
