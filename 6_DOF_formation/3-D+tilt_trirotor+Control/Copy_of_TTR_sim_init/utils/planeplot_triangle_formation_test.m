function planeplot_triangle_formation_test()
    % 动画总时间和时间步
    % 编队示意 动画显示 测试用 -- 目前 V 一键全部切换模式
    total_time = 6; % 总时间 (秒)
    dt = 0.1; % 时间步

    % 控制状态（用于暂停和加速）
    is_paused = false;  
    speed = 1;  

    % UAV 规格 (翼展 1.5m)
    b = 1.5;  
    d_x = 0.8*b; % 侧向间距
    d_y = 1*b; % 纵向间距
    h = 0.3*b; % 高度间距

    num_agents = 6; % 总 UAV 数

    % 每个 UAV 的 VTOL 模式状态（默认全部 VTOL 起飞）
    is_vtol_mode = true(1, num_agents); 

    % 编队模式选择 (1: 起降, 2: 巡航, 3: 动态任务)
    mode = 1;  

    % 初始化 UAV 状态
    position = zeros(num_agents, 3); % [X, Y, Z] 位置
    attitude = zeros(num_agents, 3); % [Roll, Pitch, Yaw] 姿态
    tilt_angle = zeros(num_agents, 2); % 电机倾转角度

    % 初始化 UAV 编队
    update_formation(mode);

    % 创建 figure
    figure;
    hold on;
    grid on;
    axis equal;
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('Multi-Mode UAV Formation Animation');
    view(3);

    setappdata(gcf, 'is_paused', is_paused);
    setappdata(gcf, 'is_vtol_mode', is_vtol_mode);
    setappdata(gcf, 'mode', mode);
    set(gcf, 'KeyPressFcn', @(src, event) keypress_callback(event));

    sim_time = 0;

    % 动画循环
    while sim_time < total_time
        is_paused = getappdata(gcf, 'is_paused');
        is_vtol_mode = getappdata(gcf, 'is_vtol_mode');
        mode = getappdata(gcf, 'mode');

        if is_paused
            pause(0.1);
            continue;
        end

        % 更新 UAV 位置
        for i = 1:num_agents
            [position(i, :), attitude(i, :)] = update_state(sim_time, total_time, position(i, :), attitude(i, :), dt, mode, is_vtol_mode(i));
        end

        % 更新每个 UAV 的电机倾转角
        for i = 1:num_agents
            if is_vtol_mode(i)
                tilt_angle(i, :) = [0, 0];  % VTOL 模式（旋翼）
            else
                tilt_angle(i, :) = [90, 90]; % 固定翼模式（巡航）
            end
        end

        % **输出 UAV 位置数据**
        fprintf('Time: %.2f sec\n', sim_time);
        for i = 1:num_agents
            fprintf('UAV %d Position: [%.2f, %.2f, %.2f] m, Tilt Angle: [%.1f°, %.1f°]\n', ...
                i, position(i, 1), position(i, 2), position(i, 3), tilt_angle(i, 1), tilt_angle(i, 2));
        end
        fprintf('-----------------------------\n');

        % 清除旧绘制
        cla;
        
        % 绘制 UAV
        for i = 1:num_agents
            planeplot_ttr(position(i, :), attitude(i, :), tilt_angle(i, :));
        end

        sim_time = sim_time + dt * speed;
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
            case '1'
                setappdata(gcf, 'mode', 1);
                update_formation(1);
                disp('Mode: Hexagonal Takeoff');
            case '2'
                setappdata(gcf, 'mode', 2);
                update_formation(2);
                disp('Mode: V-Formation Cruise');
            case '3'
                setappdata(gcf, 'mode', 3);
                update_formation(3);
                disp('Mode: Task-Based Operation');
            case 'v'
                % **单独切换每个 UAV 的 VTOL 模式**
                for i = 1:num_agents
                    is_vtol_mode(i) = ~is_vtol_mode(i);
                end
                setappdata(gcf, 'is_vtol_mode', is_vtol_mode);
                disp('VTOL mode toggled for all UAVs.');
            case 'uparrow'
                speed = speed + 1;
            case 'downarrow'
                speed = max(speed - 1, 0.1);
        end
    end

    function update_formation(mode)
        if mode == 1
            % **起降编队 - 六边形**
            angle = linspace(0, 2*pi, num_agents+1);
            for i = 1:num_agents
                position(i, :) = 1.5*[b*cos(angle(i)), b*sin(angle(i)), 0];
            end
        elseif mode == 2
            % **巡航编队 - V 形**
            position(1, :) = [ 0,   0,   0];        
            position(2, :) = [ -d_x, -d_y, h];       
            position(3, :) = [d_x, -d_y,  h];       
            position(4, :) = [-2*d_x, -2*d_y,  2*h];       
            position(5, :) = [2*d_x, -2*d_y,  2*h];       
            position(6, :) = [0, -2*d_y,  2*h];       
        elseif mode == 3
            % **任务编队 - 分组高低**
            position(1, :) = [ 0,  0,   h];       
            position(2, :) = [ d_x, d_y, -h];     
            position(3, :) = [-d_x, d_y, -h];     
            position(4, :) = [ 2*d_x, 2*d_y,  h]; 
            position(5, :) = [-2*d_x, 2*d_y, -h];
            position(6, :) = [ 0, 3*d_y,  h];    
        end
    end
end

function [new_position, new_attitude] = update_state(sim_time, total_time, position, attitude, dt, mode, is_vtol)
    speed = 1; 
    ascent_speed = 0.5;  

    if is_vtol
        % **旋翼模式：起飞阶段**
        if sim_time < total_time / 2
            position(3) = position(3) + ascent_speed * dt; % 垂直起飞
        end
    else
        % **固定翼模式：巡航**
        position(2) = position(2) + speed * dt; % 前进飞行
    end

    new_position = position;
    new_attitude = attitude;
end
