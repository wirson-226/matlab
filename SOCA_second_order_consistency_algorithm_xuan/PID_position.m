% 单个智能体的PID控制

% 初始化位置和速度
position = [0; 0];  % 初始位置
velocity = [0; 0];  % 初始速度
desired_position = [1; 1];  % 期望位置

% 控制增益
Kp = 1;  % 比例增益
Ki = 0.1;  % 积分增益
Kd = 0.5;  % 微分增益

% 初始化误差项
integral_error = [0; 0];
previous_error = desired_position - position;

% 仿真参数
dt = 0.1;  % 时间步长
simulation_time = 20;  % 仿真总时间

% 主循环进行仿真
for t = 0:dt:simulation_time
    % 计算误差
    error = desired_position - position;
    
    % 积分误差
    integral_error = integral_error + error * dt;
    
    % 微分误差
    derivative_error = (error - previous_error) / dt;
    
    % PID控制力
    control_force = Kp * error + Ki * integral_error + Kd * derivative_error;
    
    % 更新速度和位置
    velocity = velocity + control_force * dt;
    position = position + velocity * dt;
    
    % 更新上一次的误差
    previous_error = error;
    
    % 绘制当前时刻的位置
    figure(1);
    plot(position(1), position(2), 'bo', 'MarkerSize', 10);
    hold on;
    plot(desired_position(1), desired_position(2), 'ro', 'MarkerSize', 10);  % 绘制期望位置
    hold off;
    axis equal;
    title(sprintf('Time: %.1f', t));
    xlim([-2 2]);
    ylim([-2 2]);
    pause(0.1);
end
