function targets = generate_targets(t, N, mode, all_obs, r_obs)
    alpha = 1 + 0.5*sin(0.1*t);  % 可选，固定模式下可设为常数
    omega = 0.1; center = [0,0]; R = eye(2);
    
    if strcmp(mode, 'rotating')
        R = [cos(omega*t), -sin(omega*t); sin(omega*t), cos(omega*t)];
    elseif strcmp(mode, 'moving')
        center = [0.2, 0.2] * t;
    elseif strcmp(mode, 'fixed')
        alpha = 1;     % 固定大小
        R = eye(2);    % 不旋转
        center = [0, 0];  % 固定中心
    end

    raw_targets = zeros(N,2);
    for i = 1:N
        ang = 2*pi*(i-1)/N;
        p = alpha*5*[cos(ang), sin(ang)];
        raw_targets(i,:) = center + (R*p')';
    end

    % 避障修正
    offset = path_planner_avoidance(raw_targets, all_obs, r_obs);
    targets = raw_targets + offset;
end
