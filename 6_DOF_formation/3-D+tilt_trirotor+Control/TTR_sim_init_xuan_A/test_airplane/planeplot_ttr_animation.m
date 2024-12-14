function planeplot_ttr_animation()
    % 动画总时间和时间步
    total_time = 6; % 总时间 (秒)
    dt = 0.1; % 初始时间步 (秒)
    
    % 控制状态（用于暂停和加速）
    is_paused = false;  % 是否暂停
    speed = 1;  % 动画速度

    % 初始化状态
    position = [0, 0, 0]; % 初始位置
    attitude = [0, 0, 0]; % 初始姿态 [roll, pitch, yaw]
    tilt_angle = [0, 0];  % 初始电机倾转角度 [a, b]
    sim_time = 0;  % 仿真时间初始化为 0

    % 创建 figure（只创建一次）
    figure;
    hold on;
    grid on;
    axis equal;
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('Aircraft Animation');
    view(3);

    % 保存暂停状态到 figure 的属性中
    setappdata(gcf, 'is_paused', is_paused);

    % 设置键盘事件回调函数
    set(gcf, 'KeyPressFcn', @(src, event) keypress_callback(event));

    % 动画循环
    while sim_time < total_time
        % 每次循环都获取最新的暂停状态
        is_paused = getappdata(gcf, 'is_paused');
        
        % 如果暂停则跳过本次循环
        if is_paused
            pause(0.1);  % 暂停动画
            continue;  % 跳过状态更新
        end

        % 动态更新状态（模拟飞行器的动态变化）
        [position, attitude] = update_state(sim_time, total_time, position, attitude, dt);

        % 计算电机倾转角（Rotor A 和 Rotor B）
        tilt_angle(1) = 50 * sin(2 * pi * sim_time / total_time);  % Rotor A 倾转
        tilt_angle(2) = 50 * cos(2 * pi * sim_time / total_time);  % Rotor B 倾转

        % 清除之前绘制
        cla; % 只清除当前窗口的内容

        % 调用绘图函数，不再创建新窗口
        planeplot_ttr(position, attitude, tilt_angle);  % 更新同一窗口

        % 更新仿真时间
        sim_time = sim_time + dt * speed;  % 根据速度调整时间步

        % 暂停时间步
        pause(dt / speed);  % 根据当前速度调整暂停时间
    end

    function keypress_callback(event)
        % 根据按键执行不同的操作
        switch event.Key
            case 'space'
                % 按空格键切换暂停和继续
                is_paused = getappdata(gcf, 'is_paused');
                is_paused = ~is_paused;  % 切换暂停状态
                setappdata(gcf, 'is_paused', is_paused);  % 更新暂停状态
                if is_paused
                    disp('Animation Paused');
                else
                    disp('Animation Resumed');
                end
    
            case 'q'
                % 按Q键退出动画
                close(gcf);          % 关闭窗口
                disp('Animation Stopped');
    
            case 'r'
                % 按R键重置速度
                speed = 1;           
                disp('Animation speed reset to 1.');
    
            case 'uparrow'
                % 按上箭头键加速
                speed = speed + 1; 
                disp(['Speed increased to ', num2str(speed)]);
    
            case 'downarrow'
                % 按下箭头键减速
                speed = max(speed - 1, 0.1); % 最小速度限制为 0.1
                disp(['Speed decreased to ', num2str(speed)]);
    
            case 'w'
                % 按W键视图上移
                ylim(get(gca, 'YLim') + [0.1, 0.1]); 
                disp('View moved up');
    
            case 's'
                % 按S键视图下移
                ylim(get(gca, 'YLim') - [0.1, 0.1]); 
                disp('View moved down');
    
            case 'a'
                % 按A键视图左移
                xlim(get(gca, 'XLim') - [0.1, 0.1]); 
                disp('View moved left');
    
            case 'd'
                % 按D键视图右移
                xlim(get(gca, 'XLim') + [0.1, 0.1]); 
                disp('View moved right');
    
            case 'z'
                % 按Z键放大视图
                zoom(1.1); 
                disp('Zoom in');
    
            case 'x'
                % 按X键缩小视图
                zoom(0.9); 
                disp('Zoom out');
    
            otherwise
                % 未处理的按键，输出提示
                disp(['Unhandled key: ', event.Key]); 
        end
    end
end

% 更新位置和姿态状态的函数
function [new_position, new_attitude] = update_state(sim_time, total_time, position, attitude, dt)
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

    % 返回更新后的位置和姿态
    new_position = position;
    new_attitude = attitude;
end
