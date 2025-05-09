function [targets, group_centers] = generate_targets_grouped(t, group_ids, x, mode, all_obs, r_obs)
    N = length(group_ids);
    targets = zeros(N, 2);
    groups = unique(group_ids);

    % 每组分别生成一个中心
    group_centers = group_trajectory(t, length(groups));  % <--- 保留作为第二个输出

    for g = 1:length(groups)
        idx = find(group_ids == groups(g));
        n_group = length(idx);
        alpha = 1 + 0.2 * sin(0.1*t);
        r = 10; % 半径
        if strcmp(mode, 'rotating')
            omega = 50;
            R = [cos(omega*t), -sin(omega*t); sin(omega*t), cos(omega*t)];
        else
            R = eye(2);
        end
        center = group_centers(g,:);
        for k = 1:n_group
            ang = 2*pi*(k-1)/n_group;
            p = alpha * r * [cos(ang), sin(ang)];
            targets(idx(k), :) = center + (R * p')';
        end
    end

    % 避障修正
    offset = path_planner_avoidance(targets, all_obs, r_obs);
    targets = targets + offset;
end




