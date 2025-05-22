%% modules/update_moving_obstacles.m (新增/修改)
function new_pos = update_moving_obstacles(moving_obs, dt, bounds)
    % 更新动态障碍物位置，添加边界反弹
    % 输入：
    %   moving_obs - 包含pos和vel字段的结构体
    %   dt - 时间步长
    %   bounds - 边界限制 [x_min, x_max, y_min, y_max]
    
    if nargin < 3
        bounds = [-3, 3, -3, 3];  % 默认边界
    end
    if nargin < 2
        dt = 0.01;  % 默认时间步长
    end
    
    % 更新位置
    new_pos = moving_obs.pos + moving_obs.vel * dt;
    
    % 边界检查和速度反向（简单的边界反弹）
    for i = 1:size(new_pos, 1)
        % X方向边界检查
        if new_pos(i, 1) <= bounds(1) || new_pos(i, 1) >= bounds(2)
            moving_obs.vel(i, 1) = -moving_obs.vel(i, 1);
            new_pos(i, 1) = max(bounds(1), min(bounds(2), new_pos(i, 1)));
        end
        
        % Y方向边界检查
        if new_pos(i, 2) <= bounds(3) || new_pos(i, 2) >= bounds(4)
            moving_obs.vel(i, 2) = -moving_obs.vel(i, 2);
            new_pos(i, 2) = max(bounds(3), min(bounds(4), new_pos(i, 2)));
        end
    end
    
    % 更新结构体中的位置
    moving_obs.pos = new_pos;
end