% 初始化图形窗口
figure;
axis equal;
grid on;
hold on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Three Aircraft Maintaining Equilateral Triangle Formation Along Circular Path');
view(3);  % 3D视图
xlim([0 100]); ylim([0 100]); zlim([-10 10]); % 设置坐标轴范围

% 定义飞行器的尺寸
wing_span = 20;   % 翼展
fuselage_length = 10; % 机身长度
fuselage_width = 1;   % 机身宽度

% 定义飞行器顶点 (简化的三角形机翼和长方体机身)
wing_vertices = [
    -fuselage_length/2, -wing_span/2, 0;   % 左翼根
    -fuselage_length/2, wing_span/2, 0;    % 右翼根
    fuselage_length/2, 0, 0;               % 机头
];

fuselage_vertices = [
    -fuselage_length/2, -fuselage_width/2, -0.5;
    -fuselage_length/2, fuselage_width/2, -0.5;
    fuselage_length/2, fuselage_width/2, -0.5;
    fuselage_length/2, -fuselage_width/2, -0.5;
    -fuselage_length/2, -fuselage_width/2, 0.5;
    -fuselage_length/2, fuselage_width/2, 0.5;
    fuselage_length/2, fuselage_width/2, 0.5;
    fuselage_length/2, -fuselage_width/2, 0.5;
];

% 创建机翼和机身的patch对象
h_wing = gobjects(3, 1);
h_fuselage = gobjects(3, 1);
for i = 1:3
    h_wing(i) = patch('Vertices', wing_vertices, 'Faces', [1 2 3], 'FaceColor', [0.5, 0.5, 0.5], 'EdgeColor', 'none');
    h_fuselage(i) = patch('Vertices', fuselage_vertices, 'Faces', [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8], ...
                          'FaceColor', [0.7, 0.7, 0.7], 'EdgeColor', 'none');
end

% 定义飞行参数
circle_center = [50, 50, 0]; % 圆心位置
circle_radius = 30;          % 圆的半径
angular_velocity = 0.1;      % 角速度
time_step = 0.1;             % 时间步长
simulation_time = 50;        % 总仿真时间
num_steps = simulation_time / time_step; % 仿真步数

% LI制导律的增益
K_li = 2.0;  % LI制导律比例增益

% 初始姿态四元数
q_current = repmat([1, 0, 0, 0], 3, 1); % 每个飞机的四元数格式 [w, x, y, z]

% 初始化三架飞机的位置和速度
initial_angle_offset = [0, 2*pi/3, 4*pi/3]; % 等边三角形的相位偏移
current_positions = zeros(3, 3); % 每架飞机的初始位置
current_velocities = zeros(3, 3); % 每架飞机的初始速度
desired_velocities = zeros(3, 3); % 每架飞机的期望速度
desired_trajectories = zeros(num_steps, 3, 3); % 每架飞机的预期轨迹

% 设置初始位置和目标轨迹
for i = 1:3
    angle = initial_angle_offset(i);
    current_positions(i, :) = circle_center + circle_radius * [cos(angle), sin(angle), 0];
end

% 进行仿真并记录轨迹
for t = 1:num_steps
    for i = 1:3
        % 计算每架飞机的目标点 (沿圆轨迹)
        angle = initial_angle_offset(i) + angular_velocity * t * time_step;
        desired_position = circle_center + circle_radius * [cos(angle), sin(angle), 0];
        desired_trajectories(t, :, i) = desired_position; % 存储预期轨迹
        
        % 计算位置误差
        position_error = desired_position - current_positions(i, :);
        distance_to_target = norm(position_error);
        
        % LI制导律计算期望速度
        if distance_to_target > 0
            desired_velocities(i, :) = (K_li * position_error / distance_to_target); % 归一化方向向量
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
        new_wing_vertices = (R * wing_vertices')' + current_positions(i, :);
        new_fuselage_vertices = (R * fuselage_vertices')' + current_positions(i, :);
        set(h_wing(i), 'Vertices', new_wing_vertices);
        set(h_fuselage(i), 'Vertices', new_fuselage_vertices);
    end
    
    % 绘制每架飞机的预期轨迹和实际轨迹
    for i = 1:3
        plot3(desired_trajectories(1:t, 1, i), desired_trajectories(1:t, 2, i), desired_trajectories(1:t, 3, i), 'g--', 'LineWidth', 1.5); % 预期轨迹
        plot3(current_positions(i, 1), current_positions(i, 2), current_positions(i, 3), 'b.', 'MarkerSize', 3); % 实际位置
    end
    drawnow;
    
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
