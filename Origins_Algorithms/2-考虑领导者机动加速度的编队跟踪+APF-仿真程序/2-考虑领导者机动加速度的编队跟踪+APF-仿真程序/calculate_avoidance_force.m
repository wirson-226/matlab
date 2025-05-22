function F_avoid = calculate_avoidance_force(pos, obstacles, k_rep, sigma)
    F_avoid = [0; 0];
    for i = 1:size(obstacles, 1)
        x_obs = obstacles(i, 1);
        y_obs = obstacles(i, 2);
        R_obs = obstacles(i, 3);
        R_safe = obstacles(i, 4);
        
        dx = pos(1) - x_obs;
        dy = pos(3) - y_obs; % 注意状态变量为 [px, vx, py, vy]
        d = sqrt(dx^2 + dy^2);
        
        if d <= R_safe
            direction = [dx; dy] / (d + 1e-6); % 防止除以零
            magnitude = k_rep / sigma * exp(-(d - R_obs)/sigma);
            F_avoid = F_avoid + magnitude * direction;
        end
    end
end