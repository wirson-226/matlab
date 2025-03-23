clc;
clear;
close all;

%% 仿真参数设置
num_drones = 3; % 无人机数量
start_pos = [0, 0; 5, 0; 10, 0]; % 无人机起始位置 (x, y)
goal_pos = [20, 20; 25, 20; 30, 20]; % 无人机目标位置 (x, y)
obstacles = [10, 10, 2; 15, 15, 3]; % 障碍物位置和半径 (x, y, radius)
map_size = [0, 40, 0, 40]; % 地图范围 [xmin, xmax, ymin, ymax]
dt = 0.1; % 时间步长
max_time = 50; % 最大仿真时间

%% 初始化无人机状态
drones = struct();
for i = 1:num_drones
    drones(i).position = start_pos(i, :); % 当前位置
    drones(i).goal = goal_pos(i, :); % 目标位置
    drones(i).path = []; % 路径
    drones(i).velocity = [0, 0]; % 速度
end

%% 主仿真循环
figure;
hold on;
axis(map_size);
title('多无人机协同轨迹规划');
xlabel('X');
ylabel('Y');
grid on;

% 绘制障碍物
for i = 1:size(obstacles, 1)
    rectangle('Position', [obstacles(i, 1)-obstacles(i, 3), obstacles(i, 2)-obstacles(i, 3), ...
        2*obstacles(i, 3), 2*obstacles(i, 3)], 'Curvature', [1, 1], 'FaceColor', [0.5, 0.5, 0.5]);
end

% 绘制起始点和目标点标签
for i = 1:num_drones
    text(start_pos(i, 1), start_pos(i, 2), ['Start ', num2str(i)], 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
    text(goal_pos(i, 1), goal_pos(i, 2), ['Goal ', num2str(i)], 'VerticalAlignment', 'top', 'HorizontalAlignment', 'left');
end

% 仿真循环
for t = 0:dt:max_time
    cla; % 清空当前图形
    hold on;
    
    % 绘制障碍物
    for i = 1:size(obstacles, 1)
        rectangle('Position', [obstacles(i, 1)-obstacles(i, 3), obstacles(i, 2)-obstacles(i, 3), ...
            2*obstacles(i, 3), 2*obstacles(i, 3)], 'Curvature', [1, 1], 'FaceColor', [0.5, 0.5, 0.5]);
    end
    
    % 更新每架无人机的状态
    for i = 1:num_drones
        % 调用路径生成算法（预留接口）
        drones(i).path = generate_path(drones(i).position, drones(i).goal, obstacles, map_size);
        
        % 更新无人机位置（简单直线运动）
        if ~isempty(drones(i).path)
            direction = drones(i).path(1, :) - drones(i).position;
            direction = direction / norm(direction); % 单位化
            drones(i).velocity = direction * 1.5; % 假设速度为1.5 m/s
            drones(i).position = drones(i).position + drones(i).velocity * dt;
        end
        
        % 绘制无人机路径
        plot(drones(i).path(:, 1), drones(i).path(:, 2), '--', 'Color', [0, 0.5, 1]);
        plot(drones(i).position(1), drones(i).position(2), 'o', 'MarkerSize', 8, 'MarkerFaceColor', [1, 0, 0]);
    end
    
    % 绘制目标点
    for i = 1:num_drones
        plot(goal_pos(i, 1), goal_pos(i, 2), 'x', 'Color', [0, 1, 0], 'LineWidth', 2);
    end
    
    % 添加计算时间
    text(35, 35, ['Time: ', num2str(t, '%.2f'), ' s'], 'FontSize', 12, 'Color', 'black');
    
    % 刷新图形
    drawnow;
    
    % 检查是否到达目标
    if all(vecnorm([drones.position] - goal_pos, 2, 2) < 0.5)
        disp('所有无人机已到达目标！');
        break;
    end
end

%% 路径生成算法（预留接口）
function path = generate_path(start, goal, obstacles, map_size)
    % 这是一个简单的直线路径生成算法，后续可以替换为A*、RRT*、DRL等算法
    num_points = 100;
    path = [linspace(start(1), goal(1), num_points)', linspace(start(2), goal(2), num_points)'];
end
