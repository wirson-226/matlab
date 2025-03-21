% 多机 巡航 V型编队 演示效果
% 拓展 初步效果 六架编队 六边形 -- 穿插编队--分组编队2*（2+1）// （3+1） + 2

function planeplot_vformation_animation()
    % 动画总时间和时间步
    total_time = 10; % 总时间 (秒)
    dt = 0.1; % 初始时间步 (秒)
    
    % 控制状态（用于暂停和加速）
    is_paused = false;  % 是否暂停
    speed = 1;  % 动画速度

    % UAV 规格 (翼展 0.45m)
    b = 0.75; % 翼展
    d_x = 0.5*b; % 侧向间距  0.25*b - 0.5*b
    d_y = 0.8*b; % 纵向间距  0.5*b - 1.2*b
    h = 0.35*b; % 高度间距      0.1b - 0.3b
    num_rows = 5; % 每侧 5 机，+1 领队
    num_agents = 2 * num_rows + 1; % 总 UAV 数

    % 初始化 UAV 状态（所有无人机）
    position = zeros(num_agents, 3); % [X, Y, Z] 位置
    attitude = zeros(num_agents, 3); % [Roll, Pitch, Yaw] 姿态
    tilt_angle = zeros(num_agents, 2); % 电机倾转角度 [a, b]

    % % 设定 UAV 初始 V 阵列
    % for i = 1:num_rows
    %     % 左侧 UAV
    %     position(i, :) = [-i * d_x, -i * d_y, i * h];
    %     % 右侧 UAV
    %     position(num_rows + i, :) = [-i * d_x, i * d_y, i * h];
    % end
    % % 领队 UAV
    % position(end, :) = [0, 0, 0];


    % 设定 UAV 初始 V 阵列（东北天坐标系 NED）
    for i = 1:num_rows
    % 左侧 UAV（沿 X 轴负方向，Y 轴前进）
        position(i, :) = [-i * d_x, -i * d_y, i * h]; 
    % 右侧 UAV（沿 X 轴正方向，Y 轴前进）
        position(num_rows + i, :) = [i * d_x, -i * d_y, i * h]; 
    end
    % 领队 UAV（机头朝 Y 轴）
    position(end, :) = [0, 0, 0]; 


    % 创建 figure（只创建一次）
    figure;
    hold on;
    grid on;
    axis equal;
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('V-Formation UAV Animation');
    view(3);

    % 保存暂停状态到 figure 的属性中
    setappdata(gcf, 'is_paused', is_paused);

    % 设置键盘事件回调函数
    set(gcf, 'KeyPressFcn', @(src, event) keypress_callback(event));

    sim_time = 0;  % 仿真时间初始化为 0

    % 动画循环
    while sim_time < total_time
        % 获取当前的暂停状态
        is_paused = getappdata(gcf, 'is_paused');
        
        if is_paused
            pause(0.1);
            continue;
        end

        % 更新所有 UAV 状态（模拟动态编队飞行）
        for i = 1:num_agents
            [position(i, :), attitude(i, :)] = update_vformation_state(sim_time, total_time, position(i, :), attitude(i, :), dt);
        end

        % 清除之前绘制
        cla;
        
        % 绘制 UAV 编队
        for i = 1:num_agents
            planeplot_ttr(position(i, :), attitude(i, :), tilt_angle(i, :));
        end

        % 更新仿真时间
        sim_time = sim_time + dt * speed;

        % 暂停时间步
        pause(dt / speed);
    end

    function keypress_callback(event)
        switch event.Key
            case 'space'
                is_paused = getappdata(gcf, 'is_paused');
                is_paused = ~is_paused;
                setappdata(gcf, 'is_paused', is_paused);
                if is_paused, disp('Paused'); else, disp('Resumed'); end
            case 'q'
                close(gcf);
                disp('Stopped');
            case 'uparrow'
                speed = speed + 1;
                disp(['Speed: ', num2str(speed)]);
            case 'downarrow'
                speed = max(speed - 1, 0.1);
                disp(['Speed: ', num2str(speed)]);
        end
    end
end

% 更新位置和姿态状态的函数（V 形动态飞行）
function [new_position, new_attitude] = update_vformation_state(sim_time, total_time, position, attitude, dt)
    speed = 1; % 水平飞行速度
    angle_amp = 5; % 轻微机动角度变化

    % 按时间段变化（模拟队形动态调整）
    if sim_time < total_time / 4
        position(2) = position(2) + speed * dt; % Y 方向调整
    elseif sim_time < total_time / 2
        position(2) = position(2) + speed * dt; % Y 方向调整
    elseif sim_time < 4 * total_time / 4
        position(3) = position(3) + 0.2 * dt; % 轻微升降
    % else
    %     attitude(3) = attitude(3) + angle_amp * dt; % 轻微航向调整
    end

    new_position = position;
    new_attitude = attitude;
end
