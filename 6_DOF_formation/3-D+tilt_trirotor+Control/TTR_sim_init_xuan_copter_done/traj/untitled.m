% MATLAB仿真框架：多无人机协同轨迹规划（A*算法优化版）
clc;
clear;
close all;

%% 仿真参数设置
num_drones = 3; % 无人机数量
start_pos = [0, 0; 5, 0; 10, 0]; % 无人机起始位置 (x, y)
goal_pos = [50, 50; 50,50; 50, 50]; % 无人机目标位置 (x, y)
% goal_pos = [30, 30; 35, 25; 40, 35]; % 无人机目标位置 (x, y)
map_size = [0, 50, 0, 50]; % 地图范围 [xmin, xmax, ymin, ymax]
dt = 0.1; % 时间步长
max_time = 50; % 最大仿真时间
velocity_magnitude = 1.5; % 无人机速度 (m/s)
resolution = 1; % 网格分辨率

%% **生成随机障碍物**
num_obstacles = 0;  
rng(42); % 固定随机种子
obstacles = [];
goal_margin = 5; % 目标点的安全距离

while size(obstacles, 1) < num_obstacles
    x = randi([5, 45]);
    y = randi([5, 45]);
    r = randi([2, 4]); % 障碍物半径
    % 确保目标点与障碍物保持安全距离
    if size(obstacles, 1) == 0 || ...
       all(vecnorm([x, y] - obstacles(:, 1:2), 2, 2) > r + 1) && ...
       all(vecnorm([x, y] - goal_pos, 2, 2) > r + goal_margin)
        obstacles = [obstacles; x, y, r];
    end
end

%% **初始化无人机状态**
drones = struct();
colors = lines(num_drones); % 生成不同颜色
for i = 1:num_drones
    drones(i).position = start_pos(i, :);
    drones(i).goal = goal_pos(i, :);
    drones(i).path = [];
    drones(i).velocity = [0, 0];
    drones(i).reached = false;
    drones(i).history = start_pos(i, :);
end

%% **计算初始路径**
for i = 1:num_drones
    timeout = 10; % 超时限制
    tic;
    while toc < timeout
        drones(i).path = a_star(drones(i).position, drones(i).goal, obstacles, map_size, resolution);
        if ~isempty(drones(i).path)
            break;
        end
    end
    if isempty(drones(i).path)
        disp(['无人机', num2str(i), ' 无法找到路径，跳过！']);
        drones(i).reached = true;
    end
end

%% **仿真主循环**
figure;
hold on;
axis(map_size);
grid on;
xlabel('X (m)');
ylabel('Y (m)');

for t = 0:dt:max_time
    cla;
    hold on;
    title(sprintf('多无人机协同轨迹规划（A*算法） | 时间: %.1f s', t));

    % **绘制障碍物**
    for i = 1:size(obstacles, 1)
        x = obstacles(i, 1);
        y = obstacles(i, 2);
        r = obstacles(i, 3);
        theta = linspace(0, 2*pi, 20);
        fill(x + r*cos(theta), y + r*sin(theta), [0.5, 0.5, 0.5], 'FaceAlpha', 0.4, 'EdgeColor', 'k');
    end

    % **绘制目标点**
    plot(goal_pos(:, 1), goal_pos(:, 2), 'gx', 'MarkerSize', 10, 'LineWidth', 2);

    % **更新无人机状态**
    for i = 1:num_drones
        if ~drones(i).reached && ~isempty(drones(i).path)
            % **朝着路径点移动**
            target_point = drones(i).path(1, :);
            direction = target_point - drones(i).position;
            if norm(direction) < 0.5
                drones(i).path(1, :) = [];
                if isempty(drones(i).path)
                    drones(i).reached = true;
                    disp(['无人机', num2str(i), ' 已到达目标']);
                    continue;
                end
                target_point = drones(i).path(1, :);
                direction = target_point - drones(i).position;
            end
            direction = direction / norm(direction); % 归一化方向
            drones(i).velocity = direction * velocity_magnitude;
            drones(i).position = drones(i).position + drones(i).velocity * dt;
            drones(i).history = [drones(i).history; drones(i).position]; % 记录历史轨迹
        end

        % **绘制路径**
        plot(drones(i).history(:, 1), drones(i).history(:, 2), '-', 'Color', colors(i, :), 'LineWidth', 1.5);
        plot(drones(i).position(1), drones(i).position(2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', colors(i, :));

        % **显示实时坐标**
        text(drones(i).position(1) + 0.5, drones(i).position(2) + 0.5, ...
            sprintf('ID%d (%.1f, %.1f)', i, drones(i).position(1), drones(i).position(2)), 'Color', 'k', 'FontSize', 10);
    end

    drawnow;
    pause(0.01);

    % **终止条件**
    if all([drones.reached])
        disp('所有无人机已到达目标！');
        break;
    end
end

%% **A*算法**
function path = a_star(start, goal, obstacles, map_size, resolution)
    grid_x = (map_size(2) - map_size(1)) / resolution;
    grid_y = (map_size(4) - map_size(3)) / resolution;
    grid_map = zeros(grid_x, grid_y);

    % **标记障碍物**
    for i = 1:size(obstacles, 1)
        for x = max(1, obstacles(i, 1) - obstacles(i, 3)):min(grid_x, obstacles(i, 1) + obstacles(i, 3))
            for y = max(1, obstacles(i, 2) - obstacles(i, 3)):min(grid_y, obstacles(i, 2) + obstacles(i, 3))
                grid_map(x, y) = 1;
            end
        end
    end

    % **A*路径搜索**
    start_node = round(start / resolution);
    goal_node = round(goal / resolution);
    open_set = [start_node, 0, heuristic(start_node, goal_node)];
    closed_set = [];
    came_from = containers.Map();

    while ~isempty(open_set)
        [~, idx] = min(open_set(:, 3));
        current = open_set(idx, 1:2);

        if isequal(current, goal_node)
            path = reconstruct_path(came_from, current, resolution);
            return;
        end

        open_set(idx, :) = [];
        closed_set = [closed_set; current];

        neighbors = [current + [1, 0]; current + [-1, 0]; current + [0, 1]; current + [0, -1]];
        for i = 1:size(neighbors, 1)
            neighbor = neighbors(i, :);
            if any(neighbor < 1) || neighbor(1) > grid_x || neighbor(2) > grid_y || grid_map(neighbor(1), neighbor(2)) == 1
                continue;
            end
            if ismember(neighbor, closed_set, 'rows'), continue; end
            open_set = [open_set; neighbor, heuristic(start_node, neighbor), heuristic(neighbor, goal_node)];
            came_from(num2str(neighbor)) = current;
        end
    end
    path = [];
end


%% 启发式函数（曼哈顿距离）
function h = heuristic(node, goal)
    % 曼哈顿距离
    h = abs(node(1) - goal(1)) + abs(node(2) - goal(2));
end


%% **重构路径函数**
function path = reconstruct_path(came_from, current, resolution)
    path = [];
    while isKey(came_from, num2str(current))
        path = [current * resolution; path];
        current = came_from(num2str(current));
    end
    path = [current * resolution; path];
end