function planeplot_ttr_animation()
    % 动画总时间和时间步
    total_time = 6; % 总时间 (秒)
    dt = 0.1; % 时间步 (秒)
    num_steps = total_time / dt; % 总步数

    % 初始化状态
    position = [0, 0, 0]; % 初始位置
    attitude = [0, 0, 0]; % 初始姿态 [roll, pitch, yaw]
    tilt_angle = [0, 0];  % 初始电机倾转角度 [a, b]

    % 创建 figure（只创建一次）
    figure;
    hold on;
    grid on;
    axis equal;
    % xlim([-1000, 1000]);
    % ylim([-1000, 1000]);
    % zlim([-500, 500]);
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('Aircraft Animation');
    view(3);

    % 动画循环
    for step = 1:num_steps
        % 动态更新状态（模仿给出的Python代码）
        sim_time = step * dt; % 当前模拟时间

        % 根据时间段逐步修改状态
        if sim_time < total_time / 6
            position(1) = position(1) + 10 * dt;  % X方向（北向）平移
        elseif sim_time < 2 * total_time / 6
            position(2) = position(2) + 10 * dt;  % Y方向（东向）平移
        elseif sim_time < 3 * total_time / 6
            position(3) = position(3) + 10 * dt;  % Z方向（高度）增加
        elseif sim_time < 4 * total_time / 6
            attitude(3) = attitude(3) + 0.5 * dt;  % 航向角变化（Yaw）
        elseif sim_time < 5 * total_time / 6
            attitude(2) = attitude(2) + 0.5 * dt;  % 俯仰角变化（Pitch）
        else
            attitude(1) = attitude(1) + 0.5 * dt;  % 滚转角变化（Roll）
        end

        % 计算电机倾转角（Rotor A 和 Rotor B）
        tilt_angle(1) = 50 * sin(2 * pi * sim_time / total_time);  % Rotor A 倾转
        tilt_angle(2) = 50 * cos(2 * pi * sim_time / total_time);  % Rotor B 倾转



        % 清除之前绘制
        cla; % 只清除当前窗口的内容

        % 调用绘图函数，不再创建新窗口
        planeplot_ttr_test(position, attitude, tilt_angle);  % 更新同一窗口

        % 暂停时间步
        pause(dt);
    end
end
