%% 修改后的主控制接口函数
function u_total = threeFollowers_control(t, x, ctrl, all_obs)
    % 输入：
    % t - 时间
    % x - 状态矩阵 (4×4): [agent×[x,vx,y,vy]]
    % ctrl - 控制参数结构体
    % all_obs - 所有障碍物位置 (N×2): [obstacle×[x,y]]
    
    % 输出：
    % u_total - 控制输入 (4×2): [agent×[ux,uy]]
    
    % 将障碍物信息添加到ctrl结构体
    if nargin >= 4 && ~isempty(all_obs)
        % 确保obstacles格式为2×N（原协议期望的格式）
        if size(all_obs, 1) > size(all_obs, 2)
            ctrl.obstacles = all_obs';  % 转置为2×N
        else
            ctrl.obstacles = all_obs;
        end
    else
        ctrl.obstacles = [];  % 无障碍物
    end
    
    % 将状态重排为原协议格式 X(16×1)
    X = zeros(16,1);
    for i = 1:4
        X(4*(i-1)+1:4*i) = [x(i,1); x(i,2); x(i,3); x(i,4)]; % [x,vx,y,vy]
    end
    
    % 调用原协议函数
    dX_dt = threeFollowers_IAPF2022_simulation(t, X, ctrl,all_obs);
    
    % 提取控制输入（从状态导数反推）
    u_total = zeros(4, 2);
    for i = 1:4
        % 对于二阶积分器：dvx/dt = ux, dvy/dt = uy
        u_total(i, 1) = dX_dt(4*(i-1) + 2);  % ux = dvx/dt
        u_total(i, 2) = dX_dt(4*(i-1) + 4);  % uy = dvy/dt
    end
end