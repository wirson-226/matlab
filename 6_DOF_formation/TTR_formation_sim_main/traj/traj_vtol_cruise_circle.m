function [des_state] = traj_vtol_cruise_circle(t, state)
% 创建VTOL飞行器围绕操场绕圆轨迹的期望状态

% 时间参数定义
t_start = 0;       % 起始时间
t_accel = 3;       % 加速阶段时间
t_cruise = 10;     % 巡航阶段时间
t_decel = 3;       % 减速阶段时间

% 操场圆形轨迹参数
center = [100; 100; 5];  % 圆心位置 [x; y; z]
radius = 80;              % 圆半径(米)
cruise_speed = 15;        % 巡航速度(米/秒)
omega = cruise_speed / radius;  % 角速度(弧度/秒)

% 初始化期望状态
des_state = struct();

if t <= t_cruise
    % 巡航阶段 - 匀速绕圆运动
    t_elapsed = t;
    theta = omega * t_elapsed;  % 当前角度
    
    % 计算圆周上的位置
    des_state.pos = center + [radius * cos(theta); radius * sin(theta); 0];
    des_state.Va = cruise_speed;
    des_state.yaw = theta;  % 朝向圆的切线方向
    des_state.mode = 3;  % 巡航模式

else
    % 最终悬停
    theta_end = omega * t_cruise;
    des_state.pos = center + [radius * cos(theta_end); radius * sin(theta_end); 0];
    des_state.Va = 0;
    des_state.yaw = theta_end;
    des_state.mode = 1;  % 悬停模式
end

end