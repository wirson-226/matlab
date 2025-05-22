
%% main.m - 完整修正版
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

% ==== 初始化记录 ====
agent_positions = zeros(num_agents, n_dim, steps);  % 位置记录
time_stamps = (0:steps-1)' * dt;                  % 时间戳列向量
center_hist = zeros(length(group_sizes), n_dim, steps);

% ==== 视频录制 ====
record_video = true;
if record_video
    vwriter = init_video(dt); 
end

% ==== 主循环 ====
for step = 1:steps
    t = time_stamps(step);
    
    % 更新障碍物位置
    moving_obs.pos = update_moving_obstacles(moving_obs);
    all_obs = [static_obs; moving_obs.pos];
    
    % 通信拓扑更新
    adjacency = update_adjacency(x, comm_radius);
    
    % 控制计算
    u_consensus = consensus_control(x, v, d_hat, adjacency, ctrl, u_max);
    [targets, centers] = generate_targets_grouped(t, group_ids, formation_mode, x, all_obs, obstacle_radius, dt);
    center_hist(:,:,step) = centers;
    u_formation = formation_control(x, v, targets, ctrl);
    u_apf = apf_avoidance(x, all_obs, eta, d0);
    u_nominal = u_consensus + u_formation + u_apf;
    u_total = cbf_avoidance_batch(x, v, u_nominal, all_obs, obstacle_radius, r_safe, u_max, gamma);
    
    % 状态更新
    d_real = 0.3 * sin(0.1 * t) * ones(num_agents, n_dim);
    v = v + (u_total + d_real) * dt; 
    x = x + v * dt;
    
    % 记录数据
    agent_positions(:, :, step) = x;
    [state_hist, d_hist, dhat_hist] = record_history(state_hist, d_hist, dhat_hist, x, v, d_real, d_hat, step);
    
    % 可视化
    if record_video
        frame = render_frame(x, targets, all_obs, obstacle_radius, adjacency, false, group_ids, centers);
        writeVideo(vwriter, frame);
    end
end

% ==== 清理视频录制 ====
if record_video
    close(vwriter); 
end

% ==== 数据保存 ====
output_dir = fullfile(pwd, 'results');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

% 生成CSV数据
csv_data = zeros(steps, 1 + num_agents * n_dim);
csv_data(:, 1) = time_stamps;
for step = 1:steps
    pos_data = agent_positions(:, :, step)';  % n_dim × num_agents
    csv_data(step, 2:end) = pos_data(:)';    % 展平为行向量
end

% 生成列名
column_names = {'Time'};
coord_suffix = {'x','y','z'};
for i = 1:num_agents
    for d = 1:n_dim
        column_names{end+1} = sprintf('Agent%d_%s', i, coord_suffix{d});
    end
end

% 写入文件
csv_table = array2table(csv_data, 'VariableNames', column_names);
writetable(csv_table, fullfile(output_dir, 'agent_positions.csv'));

% ==== 结果绘图 ====
plot_all_results(state_hist, d_hist, dhat_hist, center_hist, targets, dt, output_dir);

disp('==== 运行完成 ====');
disp(['智能体位置数据已保存至: ' fullfile(output_dir, 'agent_positions.csv')]);
disp(['视频文件已保存至: ' fullfile(output_dir, 'simulation.mp4')]);