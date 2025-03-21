function planeplot_formation_animation()
    % 动画总时间和时间步
    total_time = 6; % 总时间 (秒)
    dt = 0.1; % 时间步

    % 控制状态（用于暂停和加速）
    is_paused = false;  
    speed = 1;  
    % is_vtol_mode = true; % 初始状态：旋翼模式（VTOL）

    % UAV 规格 (翼展 1.5m)
    b = 1.5;  
    d_x = 0.8*b; % 侧向间距
    d_y = 1*b; % 纵向间距
    h = 0.3*b; % 高度间距

    num_agents = 6; % 总 UAV 数

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
    % setappdata(gcf, 'is_vtol_mode', is_vtol_mode);
    setappdata(gcf, 'mode', mode);
    set(gcf, 'KeyPressFcn', @(src, event) keypress_callback(event));

    sim_time = 0;

    % 动画循环
    while sim_time < total_time
        is_paused = getappdata(gcf, 'is_paused');
        % is_vtol_mode = getappdata(gcf, 'is_vtol_mode');
        mode = getappdata(gcf, 'mode');

        if is_paused
            pause(0.1);
            continue;
        end

        % 更新 UAV 位置
        for i = 1:num_agents
            [position(i, :), attitude(i, :)] = update_state(sim_time, total_time, position(i, :), attitude(i, :), dt, mode);
        end

        % % 更新电机倾转角
        % if is_vtol_mode
        %     tilt_angle(i, :) = [0,0];  % 旋翼模式（VTOL）
        % else
        %     tilt_angle(i, :) = [90,90]; % 固定翼模式（巡航）
        % end

        % **输出 UAV 位置数据**
        fprintf('Time: %.2f sec\n', sim_time);
        for i = 1:num_agents
            fprintf('UAV %d Position: [%.2f, %.2f, %.2f] m, Tilt Angle: %.1f°\n', ...
                i, position(i, 1), position(i, 2), position(i, 3), tilt_angle(i, :));
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
                is_vtol_mode = ~getappdata(gcf, 'is_vtol_mode');
                setappdata(gcf, 'is_vtol_mode', is_vtol_mode);
                if is_vtol_mode
                    disp('VTOL Mode (旋翼起飞)');
                else
                    disp('Cruise Mode (固定翼巡航)');
                end
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
                tilt_angle(i, :) = [0,0];
            end
        elseif mode == 2
            % **巡航编队 - V 形**
            position(1, :) = [ 0,   0,   0];        
            position(2, :) = [ -d_x, -d_y, h];       
            position(3, :) = [d_x, -d_y,  h];       
            position(4, :) = [-2*d_x, -2*d_y,  2*h];       
            position(5, :) = [2*d_x, -2*d_y,  2*h];       
            position(6, :) = [0, -2*d_y,  2*h];   
            for i = 1:num_agents
                tilt_angle(i, :) = [90,90];
            end


        elseif mode == 3
            % **任务编队 - 分组高低**
            position(1, :) = [ 0,  0,   h];       
            position(2, :) = [ d_x, d_y, -h];     
            position(3, :) = [-d_x, d_y, -h];     
            position(4, :) = [ 2*d_x, 2*d_y,  h]; 
            position(5, :) = [-2*d_x, 2*d_y, -h];
            position(6, :) = [ 0, 3*d_y,  h];  

            tilt_angle(1, :) = [0,0];
            tilt_angle(2, :) = [0,0];
            tilt_angle(3, :) = [0,0];
            tilt_angle(4, :) = [0,0];
            tilt_angle(5, :) = [0,0];   
            tilt_angle(6, :) = [0,0];

        end
    end
end

function [new_position, new_attitude] = update_state(sim_time, total_time, position, attitude, dt, mode)
    speed = 1; 
    ascent_speed = 0.5;  

    if sim_time < total_time / 2

        position(3) = position(3) + ascent_speed * dt; % 垂直起飞
    else
        % **固定翼模式：巡航**
        position(2) = position(2) + speed * dt; % 前进飞行
    end

    new_position = position;
    new_attitude = attitude;
end
