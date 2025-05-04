%% main.m
clear; clc; close all;

addpath('modules');  % 模块目录

% ==== 参数定义 ====
[num_agents, n_dim, dt, T_total, u_max, r_safe, formation_mode, ctrl, gamma, eta, d0, comm_radius] = init_params();
steps = T_total / dt;

% ==== 分组设置 ====
group_sizes = [5, 3];  % 两组，5个 + 3个
assert(sum(group_sizes) == num_agents, '总数必须匹配');
group_ids = repelem(1:length(group_sizes), group_sizes);

[x, v, d_hat, d_hist, dhat_hist, state_hist] = init_state(num_agents, n_dim, steps, group_ids);
[static_obs, moving_obs, obstacle_radius] = init_obstacles();

record_video = true;
if record_video, vwriter = init_video(dt); end

center_hist = zeros(length(group_sizes), n_dim, steps);  % 添加中心点记录

for step = 1:steps
    t = (step - 1) * dt;
    moving_obs.pos = update_moving_obstacles(moving_obs);
    all_obs = [static_obs; moving_obs.pos];

    % adjacency = ones(num_agents) - eye(num_agents);
    adjacency = update_adjacency(x, comm_radius);

    % ==== 一致性控制 ====
    u_consensus = consensus_control(x, v, d_hat, adjacency, ctrl, u_max);

    % ==== 分组目标生成 ====
    [targets, centers] = generate_targets_grouped(t, group_ids, formation_mode, x, all_obs, obstacle_radius,dt);
    center_hist(:,:,step) = centers;

    % ==== 队形生成控制 ====
    u_formation = formation_control(x, v, targets, ctrl);

    % ==== 实际位置避障修正（实体避障） ====
    u_apf = apf_avoidance(x, all_obs, eta, d0);

    % ==== 融合控制输入阶段一 ==== 
    u_nominal = u_consensus + u_formation + u_apf;

    % ==== 阶段二 CBF优化确保最终安全输入 ====
    u_total = cbf_avoidance_batch(x, v, u_nominal, all_obs, obstacle_radius, r_safe, u_max, gamma);

    % ==== 状态更新与扰动 ==== 
    d_real = 0.3 * sin(0.1 * t) * ones(num_agents, n_dim);
    v = v + (u_total + d_real) * dt; 
    x = x + v * dt;

    % ==== 数据记录 ====
    [state_hist, d_hist, dhat_hist] = record_history(state_hist, d_hist, dhat_hist, x, v, d_real, d_hat, step);

    % ==== 可视化输出 ====
    if record_video       
        frame = render_frame(x, targets, all_obs, obstacle_radius, adjacency, false, group_ids, centers);  % 传入 group_ids 和 centers
        writeVideo(vwriter, frame);
    end

    if step == steps
        frame = render_frame(x, targets, all_obs, obstacle_radius, adjacency, true, group_ids, centers);
    end
end
if record_video, close(vwriter); end

% ==== 绘图 ====
output_dir = fullfile(pwd, 'results');
plot_all_results(state_hist, d_hist, dhat_hist, center_hist, targets, dt, output_dir);

