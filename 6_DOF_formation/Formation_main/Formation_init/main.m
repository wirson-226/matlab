%% 修改后的主循环（main.m的关键部分）
clear; clc; close all;
addpath('modules');

% 初始化参数
[num_agents, n_dim, dt, T_total, u_max, r_safe, formation_mode, ctrl, gamma, eta, d0, comm_radius] = init_params();
steps = T_total / dt;

% 初始化状态
[x, v, desired_dist, state_hist] = init_state(num_agents, n_dim, steps,ctrl);

% 初始化障碍物
[static_obs, moving_obs, obstacle_radius] = init_obstacles();

% 可视化设置
record_video = true;  % 可设置为true来录制视频
if record_video
    vwriter = VideoWriter('formation_simulation.mp4', 'MPEG-4');
    vwriter.FrameRate = 30;
    open(vwriter);
end

fprintf('开始仿真，总步数: %d\n', steps);

for step = 1:steps
    t = (step - 1) * dt;
    
    % 更新动态障碍物位置
    moving_obs.pos = update_moving_obstacles(moving_obs, dt, [-3, 3, -3, 3]);
    
    % 合并所有障碍物
    all_obs = [static_obs; moving_obs.pos];
    
    % 更新通信拓扑（如果需要）
    adjacency = update_adjacency(x, comm_radius);
    
    % === 核心控制：使用三无人机编队协议 ===
    u_total = Tracking_IAPF_control(t, x, ctrl, all_obs);

    % === 状态更新 ===
    x_next = x;
    for i = 1:num_agents
        % 二阶积分器动态：[x, vx, y, vy] -> [x+vx*dt, vx+ux*dt, y+vy*dt, vy+uy*dt]
        x_next(i, 1) = x(i, 1) + x(i, 2) * dt;        % x = x + vx*dt
        x_next(i, 2) = x(i, 2) + u_total(i, 1) * dt;  % vx = vx + ux*dt
        x_next(i, 3) = x(i, 3) + x(i, 4) * dt;        % y = y + vy*dt
        x_next(i, 4) = x(i, 4) + u_total(i, 2) * dt;  % vy = vy + uy*dt
    end
    x = x_next;
    
    % === 记录数据 ===
    state_hist(step, :, :) = x;
    
    % === 实时可视化 ===
    if mod(step, 10) == 0 || step == 1  % 每10步或第一步可视化
        fig_handle = plot_formation_state(x, all_obs, obstacle_radius, t, step == 1,ctrl);

        if record_video
            frame = getframe(fig_handle);
            writeVideo(vwriter, frame);
        end

        % 显示进度
        if mod(step, 100) == 0
            fprintf('仿真进度: %.1f%% (步数: %d/%d)\n', step/steps*100, step, steps);
        end
    end
    % 显示进度
    if mod(step, 100) == 0
        fprintf('仿真进度: %.1f%% (步数: %d/%d)\n', step/steps*100, step, steps);
    end
end



% 关闭视频录制
if record_video
    close(vwriter);
    fprintf('视频已保存为: formation_simulation.mp4\n');
end

% ==== 数据保存 ====
output_dir = fullfile(pwd, 'results');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

% === 最终结果分析和绘图 ===
% fprintf('仿真完成，正在生成结果图表...\n');
% plot_simulation_results(state_hist,static_obs, dt);

% ==== 结果绘图 ====
plot_all_results(state_hist,desired_dist,static_obs, dt, output_dir);
disp(['视频文件已保存至: ' fullfile(output_dir, 'formation_simulation.mp4')]);

