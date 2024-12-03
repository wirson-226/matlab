% 初始化图形窗口
figure;
axis equal;
grid on;
hold on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Airplane flight');
view(30,30);  % 3D视图
xlim([0 100]); ylim([0 100]); zlim([-10 10]); % 设置坐标轴范围

% 定义飞行参数
time_step = 0.1;             % 时间步长
simulation_time = 50;        % 总仿真时间
num_steps = simulation_time / time_step; % 仿真步数
num_agents = 1;

% 更新飞行路径为操场型
stadium_length = 10; % 长直线部分的长度
stadium_radius = 5;  % 半圆部分的半径
% num_segments = 4;      % 四段路径：两条直线和两条半圆

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

% 设置初始位置和目标轨迹
trajectory_lines = zeros(num_agents, 1); % 用于存储轨迹句柄


% 进行仿真并记录轨迹
for t = 1:num_steps
    for i = 1:num_agents
        % 计算每架飞机的目标点 (沿圆轨迹)
        % 计算当前时间段的位置（根据路径段选择）
        path_position = mod(t * time_step, stadium_length + 2*pi*stadium_radius);  % 总路径长度

        if path_position < stadium_length % 在第一条直线部分
            desired_position = [path_position, 0, 0]; % 沿x轴的直线
        elseif path_position < stadium_length + pi*stadium_radius % 在第一个半圆部分
            angle = (path_position - stadium_length) / stadium_radius; % 计算角度
            desired_position = stadium_radius * [cos(angle), sin(angle), 0];
        elseif path_position < 2*stadium_length + pi*stadium_radius % 在第二条直线部分
            desired_position = [stadium_length, path_position - stadium_length - pi*stadium_radius, 0];
        else % 在第二个半圆部分
            angle = (path_position - 2*stadium_length - pi*stadium_radius) / stadium_radius;
            desired_position = stadium_radius * [cos(angle), sin(angle), 0];
        end

        % 存储预期轨迹
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
        cla; % 只清除当前窗口的内容
        planeplot_ttr_test(current_positions(i, :), att, tilt);  % 更新同一窗口

    % % 绘制每架飞机的预期轨迹和实际轨迹

        plot3(desired_trajectories(1:t, 1, i), desired_trajectories(1:t, 2, i), desired_trajectories(1:t, 3, i), 'g--', 'LineWidth', 1.5); % 预期轨迹
        plot3(current_trajectories(1:t, 1, i), current_trajectories(1:t, 2, i), current_trajectories(1:t, 3, i), 'b', 'LineWidth', 1); % 实际位置
    end
    drawnow;  % 更新图形
    
    % 添加动态更新（可选）
    pause(0.05);
end

% 绘制参考坐标系
quiver3(0, 0, 0, 10, 0, 0, 'r', 'LineWidth', 2);  % X轴
quiver3(0, 0, 0, 0, 10, 0, 'g', 'LineWidth', 2);  % Y轴
quiver3(0, 0, 0, 0, 0, 10, 'b', 'LineWidth', 2);  % Z轴

% 显示图形
hold off;





% 辅助函数：四元数插值 (Slerp)
function q_interp = slerp(q1, q2, t)
    % 计算四元数的点积
    dot_product = dot(q1, q2);
    
    % 如果点积为负，反转四元数
    if dot_product < 0
        q1 = -q1;
        dot_product = -dot_product;
    end
    
    % 确保点积在有效范围内
    dot_product = min(1.0, max(-1.0, dot_product));
    
    % 计算插值角度
    theta_0 = acos(dot_product);
    sin_theta_0 = sin(theta_0);
    
    % 如果角度非常小，则使用线性插值
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
