% modules/generate_targets.m
function targets = generate_targets(t, N, mode, all_obs, r_obs)
    alpha = 1 + 0.5*sin(0.1*t);  % 可选，固定模式下可设为常数
    omega = 0.1; R = eye(2);

    % 引入参考轨迹中心
    center = reference_trajectory(t);

    if strcmp(mode, 'rotating')
        R = [cos(omega*t), -sin(omega*t); sin(omega*t), cos(omega*t)];
    elseif strcmp(mode, 'fixed')
        alpha = 1;     % 固定大小
        R = eye(2);    % 不旋转
    elseif strcmp(mode, 'moving')
        % 已在 reference_trajectory 中体现
    end

    % 定义标准五边形或圆形结构位置点
    raw_targets = zeros(N,2);
    for i = 1:N
        ang = 2*pi*(i-1)/N;
        p = alpha*5*[cos(ang), sin(ang)];
        raw_targets(i,:) = center + (R*p')';
    end

    % 避障修正（可选）
    offset = path_planner_avoidance(raw_targets, all_obs, r_obs);
    targets = raw_targets + offset;
end

