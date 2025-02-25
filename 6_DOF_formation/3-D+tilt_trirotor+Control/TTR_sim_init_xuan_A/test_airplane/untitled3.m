%%% 固定翼模式轨迹跟踪方法 %%%

% 初始化图形窗口
figure;
axis equal;
grid on;
hold on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Airplane flight');
view(3);  % 3D视图
xlim([-100 100]); ylim([-100 100]); zlim([-10 10]); % 设置坐标轴范围

% 定义飞行参数
time_step = 0.1;             % 时间步长
simulation_time = 500;        % 总仿真时间
num_steps = simulation_time / time_step; % 仿真步数
num_agents = 3;  % 三架飞机

% 更新飞行路径为操场型
stadium_length = 10; % 长直线部分的长度
stadium_radius = 5;  % 半圆部分的半径

% LI制导律的增益
K_li = 2;  % LI制导律比例增益

% 初始姿态四元数
q_current = repmat([1, 0, 0, 0], num_agents, 1); % 每个飞机的四元数格式 [w, x, y, z]

% 飞机编队初始位置（等边三角形）
formation_offset = [0, 10, 0; -5, -5, 0; 5, -5, 0];  % 相对位置偏移 (3x3)，包括z轴偏移
base_position = repmat([-20, 0, 0], 3, 1);  % 将基准位置扩展为3x3矩阵

% 将基础位置添加到每架飞机的偏移
current_positions = base_position + formation_offset;  % 编队位置

% 现在每架飞机的 `z` 坐标已设为 0
current_positions(:, 3) = 0;  % 确保每架飞机的z轴位置为0

current_velocities = zeros(num_agents, 3); % 每架飞机的初始速度
desired_velocities = zeros(num_agents, 3); % 每架飞机的期望速度
desired_trajectories = zeros(num_steps, 3, num_agents); % 每架飞机的预期轨迹
current_trajectories = zeros(num_steps, 3, num_agents); % 每架飞机的实际轨迹

% 飞机的目标视角距离
camera_distance = -10;

% 绘制参考坐标系
quiver3(0, 0, 0, 10, 0, 0, 'r', 'LineWidth', 2);  % X轴
quiver3(0, 0, 0, 0, 10, 0, 'g', 'LineWidth', 2);  % Y轴
quiver3(0, 0, 0, 0, 0, 10, 'b', 'LineWidth', 2);  % Z轴

%% 杜宾曲线生成路径
% 点集合，每一行为一个点，包含 x, y, theta
points = [
    5, -5, pi/4;
    10, 0, pi/3;
    15, 5, -pi/4;
    10, 10, pi/6;
    5, 15, 0
];

% 最小转弯半径
r = 10;
h = 0; % 高度（平面内保持不变）
% 步长（决定路径精细程度）
stepsize = 0.1;
% 是否安静模式，0 表示显示输出
quiet = 1;
% 调用生成路径函数
full_path = generate_dubins_path(points, r, stepsize, h, quiet);
% 提取路径的 x, y 坐标
traj_xy = [full_path(:,1), full_path(:,2)];

% 编队控制增益
K_pos = 2;   % 位置控制增益
K_vel = 0.5;  % 速度控制增益
damping = 0.1;  % 阻尼系数，防止过快的响应

% 进行仿真并记录轨迹
for t = 1:num_steps
    for i = 1:num_agents
        % 存储预期轨迹
        desired_position = [traj_xy(t, 1), traj_xy(t, 2), h];
        desired_trajectories(t, :, i) = desired_position;
        current_trajectories(t, :, i) = current_positions(i, :);
        
        % 计算位置误差和期望速度
        [position_error, distance_to_target] = compute_position_error(desired_position, current_positions(i, :));
        
        % LI制导律计算期望速度
        if distance_to_target > 0
            desired_velocities(i, :) = (K_li * position_error / distance_to_target); % 归一化方向向量
        end
        
        % 排斥力部分
        if distance_to_target < 10  % 设定排斥力的临界距离为10
            repulsion_force = (1 / distance_to_target^2) * position_error / distance_to_target;
            desired_velocities(i, :) = desired_velocities(i, :) - repulsion_force; % 添加排斥力
        end
        
        % 计算期望位置的偏差，并应用编队控制
        target_position = calculate_formation_target(i, current_positions, formation_offset);
        formation_error = target_position - current_positions(i, :);

        % 速度调节：根据误差大小控制速度
        velocity_error = K_pos * formation_error;
        velocity_magnitude = norm(velocity_error);

        % 使用阻尼项减缓过度的速度响应
        velocity_error = velocity_error * min(1, distance_to_target / (2 * velocity_magnitude + damping));
        
        % 更新飞行器的位置和速度
        current_velocities(i, :) = velocity_error; % 控制速度
        current_positions(i, :) = current_positions(i, :) + current_velocities(i, :) * time_step;

        % 绘制飞机模型和轨迹
        planeplot_ttr(current_positions(i, :), [0, 0, 0], [90, 90]);
        plot3(desired_trajectories(1:t, 1, i), desired_trajectories(1:t, 2, i), desired_trajectories(1:t, 3, i), 'g--', 'LineWidth', 1.5); % 预期轨迹
        plot3(current_trajectories(1:t, 1, i), current_trajectories(1:t, 2, i), current_trajectories(1:t, 3, i), 'b', 'LineWidth', 1);
    end

    % 更新相机位置：根据三架飞机的质心进行视角跟随
    camera_target = mean(current_positions, 1);  % 相机视角跟随三架飞机的质心
    camera_position = camera_target + [camera_distance-2, -10, 20];  % 设置相机位置
    campos(camera_position);          % 设置相机位置

    drawnow;  % 更新图形
    % 清除之前绘制
    cla; % 只清除当前窗口的内容
end

hold off;

%% 辅助函数：计算位置误差
function [error, distance] = compute_position_error(desired_position, current_position)
    error = desired_position - current_position;
    distance = norm(error);
end

% 辅助函数：计算编队目标位置
function target_position = calculate_formation_target(i, current_positions, formation_offset)
    target_position = current_positions(i, :) + formation_offset(i, :);
end
