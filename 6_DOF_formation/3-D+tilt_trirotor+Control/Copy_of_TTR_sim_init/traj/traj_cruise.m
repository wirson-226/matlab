function [des_state] = traj_cruise(t, state)
% 创建固定翼模式 基于dubin曲线的轨迹测试
% 输入参数:
%   t - 当前时间步（整数索引）
%   state - 当前状态（未使用）
% 输出参数:
%   des_state - 期望状态，包含位置、速度、加速度、朝向等
params = sys_params;

% 点集合，每一行为一个点，包含 x, y, theta
points = [
    0, 50,  0;
    0, 100, 0;
    0, 150, 0;
    0, 200, 0;
    0, 250, 0
];

% 最小转弯半径
r = params.R_min;
h = 10; % 高度（平面内保持不变）
% 步长（决定路径精细程度）
stepsize = 0.1;
% 是否安静模式，1 表示不显示输出
quiet = 1;

% 调用生成路径函数
full_path = generate_dubins_path(points, r, stepsize, h, quiet);

% 限制t在有效范围内
t = max(1, min(t, size(full_path, 1)));

% 提取位置和朝向
pos = full_path(t, 1:3)';
yaw = full_path(t, 3);

% 计算速度（使用前后点的差分）
if t > 1 && t < size(full_path, 1)
    vel = (full_path(t+1, 1:3) - full_path(t-1, 1:3))' / (2*stepsize);
else
    if t == 1
        vel = (full_path(2, 1:3) - full_path(1, 1:3))' / stepsize;
    else
        vel = (full_path(t, 1:3) - full_path(t-1, 1:3))' / stepsize;
    end
end

% 计算加速度（简单的二阶差分）
if t > 2 && t < size(full_path, 1)-1
    acc = (full_path(t+1, 1:3) - 2*full_path(t, 1:3) + full_path(t-1, 1:3))' / (stepsize^2);
else
    acc = [0; 0; 0];
end

% 计算yaw速率
if t > 1 && t < size(full_path, 1)
    yawdot = (full_path(t+1, 3) - full_path(t-1, 3)) / (2*stepsize);
else
    yawdot = 0;
end

% 设置输出状态
des_state.pos = pos;
des_state.vel = vel;
des_state.acc = acc;
des_state.yaw = yaw;
des_state.yawdot = yawdot;
des_state.Va = 10; % 匀速巡航
des_state.mode = 3; % 固定翼模式

end



function full_path = generate_dubins_path(points, r, stepsize, h, quiet)
% 生成连接多个带方向点的Dubins路径
% 输入:
%   points - 点集合，每行为 [x, y, theta]
%   r - 最小转弯半径
%   stepsize - 路径采样步长
%   h - 高度值（Z坐标）
%   quiet - 是否安静模式，不显示输出
% 输出:
%   full_path - 完整路径点集 [x, y, z, theta]

full_path = [];
num_points = size(points, 1);

% 至少需要两个点
if num_points < 2
    error('至少需要两个点来生成路径');
end

% 连接每对相邻点
for i = 1:(num_points-1)
    p1 = points(i, :);
    p2 = points(i+1, :);
    
    % 调用Dubins曲线函数生成路径
    path = dubins_curve(p1, p2, r, stepsize, quiet);
    
    % 添加高度坐标
    path_with_z = [path(:,1:2), repmat(h, size(path,1), 1), path(:,3)];
    
    % 如果不是第一段，移除起点（避免重复）
    if i > 1
        path_with_z = path_with_z(2:end, :);
    end
    
    % 添加到完整路径
    full_path = [full_path; path_with_z];
end

end