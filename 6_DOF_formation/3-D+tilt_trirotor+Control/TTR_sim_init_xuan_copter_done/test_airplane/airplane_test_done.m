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
num_agents = 1;

% 更新飞行路径为操场型
stadium_length = 10; % 长直线部分的长度
stadium_radius = 5;  % 半圆部分的半径

% LI制导律的增益
K_li = 2;  % LI制导律比例增益

% 初始姿态四元数
q_current = repmat([1, 0, 0, 0], 3, 1); % 每个飞机的四元数格式 [w, x, y, z]

% 初始化三架飞机的位置和速度
current_positions = [-20,-10,0;
                     0,0,0;
                     0,0,0]; % 每架飞机的初始位置
current_velocities = zeros(3, 3); % 每架飞机的初始速度
desired_velocities = zeros(3, 3); % 每架飞机的期望速度
desired_trajectories = zeros(num_steps, 3, 3); % 每架飞机的预期轨迹
current_trajectories = zeros(num_steps, 3, 3); % 每架飞机的实际轨迹

% 飞机的目标视角距离
camera_distance = -10;

% 设置视角目标点
camera_target = current_positions(1, :);

% 设置初始位置和目标轨迹
trajectory_lines = zeros(num_agents, 1); % 用于存储轨迹句柄

% 绘制参考坐标系
quiver3(0, 0, 0, 10, 0, 0, 'r', 'LineWidth', 2);  % X轴
quiver3(0, 0, 0, 0, 10, 0, 'g', 'LineWidth', 2);  % Y轴
quiver3(0, 0, 0, 0, 0, 10, 'b', 'LineWidth', 2);  % Z轴


%%杜宾曲线
% 点集合，每一行为一个点，包含 x, y, theta
points = [
    5, -5, pi/4;
    10, 0, pi/3;
    15, 5, -pi/4;
    10, 10, pi/6;
    5, 15, 0
];

% 最小转弯半径
r = 2;
h = 0; % 高度（平面内保持不变）
% 步长（决定路径精细程度）
stepsize = 0.1;
% 是否安静模式，0 表示显示输出
quiet = 1;
% 调用生成路径函数
full_path = generate_dubins_path(points, r, stepsize, h, quiet);
% 提取路径的 x, y 坐标
traj_xy = [full_path(:,1), full_path(:,2)];






% 进行仿真并记录轨迹
for t = 1:num_steps    
    for i = 1:num_agents

        % 存储预期轨迹
        desired_position = [traj_xy(t, 1), traj_xy(t, 2), h];
        desired_trajectories(t, :, i) = desired_position;
        current_trajectories(t, :, i) = current_positions(i, :);
        
        % 计算位置误差
        position_error = desired_position - current_positions(i, :);
        distance_to_target = norm(position_error);
        
        % LI制导律计算期望速度
        if distance_to_target > 0
            desired_velocities(i, :) = (K_li * position_error / distance_to_target); % 归一化方向向量
        end
        
        % 排斥力部分
        if distance_to_target < 10  % 设定排斥力的临界距离为10
            repulsion_force = (1 / distance_to_target^2) * position_error / distance_to_target;
            desired_velocities(i, :) = desired_velocities(i, :) - repulsion_force; % 添加排斥力
        end
        
        % 更新飞行器的位置和速度
        current_velocities(i, :) = desired_velocities(i, :);
        current_positions(i, :) = current_positions(i, :) + current_velocities(i, :) * time_step;
        
        % 计算期望偏航角（使用四元数表示）
        desired_yaw = atan2(desired_position(2) - current_positions(i, 2), desired_position(1) - current_positions(i, 1)); % 期望偏航角
        q_desired = [cos(desired_yaw/2), 0, 0, sin(desired_yaw/2)]; % 期望四元数
        
        % 四元数插值（Slerp）来平滑地从当前姿态过渡到期望姿态
        q_current(i, :) = slerp(q_current(i, :), q_desired, 0.1); % 使用0.1作为插值因子
        
        % 将四元数转换为旋转矩阵
        R = quat2rotm(q_current(i, :)); % 使用四元数生成旋转矩阵
        att = [0,0,desired_yaw];
        tilt = [90,90];

        % 清除之前绘制
        cla; % 只清除当前窗口的内容

        % 绘制飞机模型
        planeplot_ttr(current_positions(i, :), att, tilt);  % 更新同一窗口

        % 绘制每架飞机的预期轨迹和实际轨迹
        plot3(desired_trajectories(1:t, 1, i), desired_trajectories(1:t, 2, i), desired_trajectories(1:t, 3, i), 'g--', 'LineWidth', 1.5); % 预期轨迹
        plot3(current_trajectories(1:t, 1, i), current_trajectories(1:t, 2, i), current_trajectories(1:t, 3, i), 'b', 'LineWidth', 1);
        
        % % 更新视角：相机位置始终跟随飞机
        camera_target = current_positions(i, :);  % 相机始终跟随飞机
        camera_position = camera_target + [camera_distance-2, -10, 20];  % 设置相机位置，稍微偏离目标（例如20单位远）
        campos(camera_position);          % 设置相机位置

    end
    drawnow;  % 更新图形
end

hold off;

% 辅助函数：四元数插值 (Slerp)
function q_interp = slerp(q1, q2, t)
    dot_product = dot(q1, q2);
    if dot_product < 0
        q1 = -q1;
        dot_product = -dot_product;
    end
    dot_product = min(1.0, max(-1.0, dot_product));
    theta_0 = acos(dot_product);
    sin_theta_0 = sin(theta_0);
    
    if sin_theta_0 < 1e-6
        q_interp = (1 - t) * q1 + t * q2;
    else
        theta = theta_0 * t;
        sin_theta = sin(theta);
        
        s0 = cos(theta) - dot_product * sin_theta / sin_theta_0;
        s1 = sin_theta / sin_theta_0;
        
        q_interp = s0 * q1 + s1 * q2;
    end
end

% 辅助函数：四元数转旋转矩阵
function R = quat2rotm(q)
    % 将四元数转换为旋转矩阵
    q0 = q(1);
    q1 = q(2);
    q2 = q(3);
    q3 = q(4);
    
    R = [1-2*(q2^2+q3^2), 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
         2*(q1*q2+q0*q3), 1-2*(q1^2+q3^2), 2*(q2*q3-q0*q1);
         2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), 1-2*(q1^2+q2^2)];
end


