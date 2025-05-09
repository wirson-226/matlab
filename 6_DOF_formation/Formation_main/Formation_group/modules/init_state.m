% modules/init_state.m
function [x, v, d_hat, d_hist, dhat_hist, state_hist] = init_state(num_agents, n_dim, steps, group_ids)
    % 获取分组信息
    groups = unique(group_ids);
    num_groups = length(groups);
    x = zeros(num_agents, n_dim);

    % 每组分布在不同中心附近
    base_centers = [10, 10; -10, 10; -10, -10; 10, -10];  % 最多支持4组，可扩展
    if num_groups > size(base_centers, 1)
        error('基础中心数量不足，请扩展 base_centers');
    end

    for i = 1:num_groups
        idx = find(group_ids == groups(i));
        n_i = length(idx);
        angle = 2 * pi * rand(n_i, 1);
        radius = 5 + 5*rand(n_i, 1);  % 距离中心2~3m内随机分布
        center = base_centers(i, :);
        x(idx, :) = center + [radius .* cos(angle), radius .* sin(angle)];
    end

    v = zeros(num_agents, n_dim);
    d_hat = zeros(num_agents, n_dim);
    d_hist = zeros(num_agents, n_dim, steps);
    dhat_hist = d_hist;
    state_hist.pos = zeros(num_agents, n_dim, steps);
    state_hist.vel = state_hist.pos;
end
