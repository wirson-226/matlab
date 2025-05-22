clc
clear
close all

% --- 系统参数 ---
n = 4;
l = 0.1;
K1 = kron(eye(2), [-2, -4]);
A = kron(eye(2), [1, l; 0, 1]);
B = kron(eye(2), [0; l]);
P = dare(A + B * K1, B, eye(4), eye(2));
K = -inv(B' * P * B + eye(2)) * B' * P * (A + B * K1);
L = [1 0 0 -1;
    -1 1 0 0;
    0 -1 1 0;
    0 0 -1 1];

% --- APF 和涡旋场参数 ---
K3 = 0.03;  % 障碍物斥力增益
K4 = 0.005; % 智能体间斥力增益

% --- 涡旋场增益系数 (关键对比参数) ---
% Kv_obstacle = 0;   % 关闭障碍物涡旋场
% Kv_agent = 0;      % 关闭智能体间涡旋场

Kv_obstacle = 0.5; % 开启障碍物涡旋场
Kv_agent = 0.3;    % 开启智能体间涡旋场

te_total = 400; % 仿真总步数 (40秒)

% --- 场景选择与设置 ---
% 选择一个场景进行测试，取消注释对应的场景块

% --- 场景A：狭缝/一排紧密障碍物 (Slalom) ---
obstacle_radius = 0.5;
[xo, yo, ~] = cylinder(obstacle_radius, 50);
p_obs_centers = { [-1; 1.5], [0; 0], [1; -1.5] }; % 三个错开的障碍物形成S型路径
% 或者更直接的一排障碍物形成狭缝：
% p_obs_centers = { [0; 1.5], [0; -1.5] }; % 形成一个垂直的狭缝
% p_obs_centers = { [-1.5; 0], [1.5; 0] }; % 形成一个水平的狭缝
num_static_obstacles = length(p_obs_centers);
d_safe = 1.0; % 斥力影响距离
b = 4;        
d0 = 1.2;     
r_formation_agents = 0.8; % 编队半径，需要小于狭缝宽度
initial_formation_center_x = -5;
initial_formation_center_y = 0; % 根据狭缝方向调整
final_formation_center_x = 5;
final_formation_center_y = 0; % 根据狭缝方向调整
axis_limits = [-6 6 -3 3];

% % --- 场景B：凹形陷阱 (类似U型，但更具挑战性) ---
% obstacle_radius = 0.8;
% [xo, yo, ~] = cylinder(obstacle_radius, 50);
% p_obs_centers = { [-2; 0], [0; -2], [2; 0], [0;1.5] }; % 四个障碍物形成一个更封闭的凹形
% num_static_obstacles = length(p_obs_centers);
% d_safe = 1.2; 
% b = 4;        
% d0 = 1.5;     
% r_formation_agents = 0.7; 
% initial_formation_center_x = 0;
% initial_formation_center_y = -4; % 从陷阱开口下方进入
% final_formation_center_x = 0;
% final_formation_center_y = 3;   % 目标在陷阱内部或另一侧
% axis_limits = [-5 5 -5 4];


% --- 智能体初始位置 (形成初始编队) ---
x0 = zeros(4*n, 1);
for i = 1:n
    relative_angle_init = 2*pi*(i-1)/n;
    x0(4*i-3) = initial_formation_center_x + r_formation_agents * cos(relative_angle_init);
    x0(4*i-1) = initial_formation_center_y + r_formation_agents * sin(relative_angle_init);
    x0(4*i-2) = 0; % vx
    x0(4*i)   = 0; % vy
end

% --- 动态目标轨迹 h (编队中心直线移动) ---
h = zeros(4*n, te_total + 1);
t_vec_h = (0:te_total)*l; 
w_formation_rotation = 0.0; % 编队自身不旋转

for k_time = 1:(te_total + 1)
    current_time_h = t_vec_h(k_time);
    progress = min(1, current_time_h / (te_total*l * 0.9)); % 用90%的时间完成移动
    current_center_x = initial_formation_center_x + (final_formation_center_x - initial_formation_center_x) * progress;
    current_center_y = initial_formation_center_y + (final_formation_center_y - initial_formation_center_y) * progress;
    
    if k_time > 1 && progress < 1
        prev_center_x = initial_formation_center_x + (final_formation_center_x - initial_formation_center_x) * min(1, t_vec_h(k_time-1) / (te_total*l * 0.9));
        prev_center_y = initial_formation_center_y + (final_formation_center_y - initial_formation_center_y) * min(1, t_vec_h(k_time-1) / (te_total*l * 0.9));
        current_center_vx = (current_center_x - prev_center_x) / l;
        current_center_vy = (current_center_y - prev_center_y) / l;
    else
        current_center_vx = (final_formation_center_x - initial_formation_center_x) / (te_total*l * 0.9); 
        current_center_vy = (final_formation_center_y - initial_formation_center_y) / (te_total*l * 0.9);
        if progress >=1 
            current_center_vx = 0; current_center_vy = 0;
        end
    end
    
    current_formation_angle_offset = w_formation_rotation * current_time_h;
    for i = 1:n
        relative_angle = 2*pi*(i-1)/n + current_formation_angle_offset;
        h(4*i-3, k_time) = current_center_x + r_formation_agents * cos(relative_angle);
        h(4*i-2, k_time) = current_center_vx - r_formation_agents * w_formation_rotation * sin(relative_angle);
        h(4*i-1, k_time) = current_center_y + r_formation_agents * sin(relative_angle);
        h(4*i, k_time)   = current_center_vy + r_formation_agents * w_formation_rotation * cos(relative_angle);
    end
end

% --- 初始化状态矩阵 ---
x = zeros(4*n, te_total + 1);
x(:, 1) = x0;

% --- 主仿真循环 (斥力/涡旋力计算 和 状态更新逻辑保持不变) ---
for k = 1:te_total
    F_Obstacles_Resultant = zeros(2*n, 1);
    F_Agents_Resultant = zeros(2*n, 1);

    % 为静态障碍物计算斥力/涡旋场力
    for i = 1:n 
        agent_pos_i = [x(4*i-3, k); x(4*i-1, k)];       
        target_pos_i = [h(4*i-3, k); h(4*i-1, k)];     
        AF_vec_i = target_pos_i - agent_pos_i; 
        current_agent_total_obs_force_x = 0;
        current_agent_total_obs_force_y = 0;
        for obs_idx = 1:num_static_obstacles 
            obs_center_j = p_obs_centers{obs_idx};
            dist_to_center_vec = agent_pos_i - obs_center_j;
            dist_to_center = norm(dist_to_center_vec);
            min_dist_ij = dist_to_center - obstacle_radius;
            Fr_x_orig_ij = 0; Fr_y_orig_ij = 0; 
            if min_dist_ij < d_safe && min_dist_ij > 1e-3 
                rep_magnitude = (b / min_dist_ij - 1 / d_safe)^2 * (b / (min_dist_ij^2));
                dir_vec_obs_to_agent_ij = dist_to_center_vec / dist_to_center; 
                Fr_x_orig_ij = rep_magnitude * dir_vec_obs_to_agent_ij(1);
                Fr_y_orig_ij = rep_magnitude * dir_vec_obs_to_agent_ij(2);
                if Kv_obstacle > 0 
                    obs_surface_point_j_for_AO = obs_center_j; 
                    AO_vec_ij = obs_surface_point_j_for_AO - agent_pos_i; 
                    Rz_ij = AF_vec_i(1) * AO_vec_ij(2) - AF_vec_i(2) * AO_vec_ij(1);
                    if Rz_ij <= 0
                        Fv_x_ij = Fr_x_orig_ij + Kv_obstacle * Fr_y_orig_ij;
                        Fv_y_ij = Fr_y_orig_ij - Kv_obstacle * Fr_x_orig_ij;
                    else 
                        Fv_x_ij = Fr_x_orig_ij - Kv_obstacle * Fr_y_orig_ij;
                        Fv_y_ij = Fr_y_orig_ij + Kv_obstacle * Fr_x_orig_ij;
                    end
                    current_agent_total_obs_force_x = current_agent_total_obs_force_x + Fv_x_ij;
                    current_agent_total_obs_force_y = current_agent_total_obs_force_y + Fv_y_ij;
                else 
                    current_agent_total_obs_force_x = current_agent_total_obs_force_x + Fr_x_orig_ij;
                    current_agent_total_obs_force_y = current_agent_total_obs_force_y + Fr_y_orig_ij;
                end
            end
        end
        F_Obstacles_Resultant(2*i-1, 1) = current_agent_total_obs_force_x;
        F_Obstacles_Resultant(2*i, 1)   = current_agent_total_obs_force_y;
    end

    % 为智能体间避碰计算斥力/涡旋场力
    for i = 1:n 
        agent_pos_i = [x(4*i-3, k); x(4*i-1, k)];
        target_pos_i = [h(4*i-3, k); h(4*i-1, k)]; 
        AF_vec_i = target_pos_i - agent_pos_i;
        current_agent_total_agent_force_x = 0;
        current_agent_total_agent_force_y = 0;
        for j = 1:n 
            if i == j; continue; end
            agent_pos_j = [x(4*j-3, k); x(4*j-1, k)]; 
            dist_ij_vec = agent_pos_i - agent_pos_j;
            dist_ij = norm(dist_ij_vec);
            Fr_x_orig_agent_ij = 0; Fr_y_orig_agent_ij = 0;
            if dist_ij < d0 && dist_ij > 1e-3 
                rep_magnitude_agent = (b / exp(dist_ij) - 1 / exp(d0))^2 * (b / exp(dist_ij)); 
                dir_vec_j_to_i = dist_ij_vec / dist_ij; 
                Fr_x_orig_agent_ij = rep_magnitude_agent * dir_vec_j_to_i(1);
                Fr_y_orig_agent_ij = rep_magnitude_agent * dir_vec_j_to_i(2);
                if Kv_agent > 0
                    AO_vec_agent_ij = agent_pos_j - agent_pos_i; 
                    Rz_agent_ij = AF_vec_i(1) * AO_vec_agent_ij(2) - AF_vec_i(2) * AO_vec_agent_ij(1);
                    if Rz_agent_ij <= 0
                        Fv_x_agent_ij = Fr_x_orig_agent_ij + Kv_agent * Fr_y_orig_agent_ij;
                        Fv_y_agent_ij = Fr_y_orig_agent_ij - Kv_agent * Fr_x_orig_agent_ij;
                    else 
                        Fv_x_agent_ij = Fr_x_orig_agent_ij - Kv_agent * Fr_y_orig_agent_ij;
                        Fv_y_agent_ij = Fr_y_orig_agent_ij + Kv_agent * Fr_x_orig_agent_ij;
                    end
                    current_agent_total_agent_force_x = current_agent_total_agent_force_x + Fv_x_agent_ij;
                    current_agent_total_agent_force_y = current_agent_total_agent_force_y + Fv_y_agent_ij;
                else
                    current_agent_total_agent_force_x = current_agent_total_agent_force_x + Fr_x_orig_agent_ij;
                    current_agent_total_agent_force_y = current_agent_total_agent_force_y + Fr_y_orig_agent_ij;
                end
            end
        end
        F_Agents_Resultant(2*i-1, 1) = current_agent_total_agent_force_x;
        F_Agents_Resultant(2*i, 1)   = current_agent_total_agent_force_y;
    end
    
    u_ff_accel = zeros(2*n, 1); 
    for agent_idx = 1:n
        if k+1 <= size(h, 2)
            desired_vel_x_k = h(4*agent_idx-2, k);
            desired_vel_x_k_plus_1 = h(4*agent_idx-2, k+1);
            desired_accel_x = (desired_vel_x_k_plus_1 - desired_vel_x_k) / l;
            desired_vel_y_k = h(4*agent_idx, k);
            desired_vel_y_k_plus_1 = h(4*agent_idx, k+1);
            desired_accel_y = (desired_vel_y_k_plus_1 - desired_vel_y_k) / l;
        else
            desired_accel_x = 0; desired_accel_y = 0;
        end
        u_ff_accel(2*agent_idx-1) = desired_accel_x; 
        u_ff_accel(2*agent_idx)   = desired_accel_y; 
    end
    
    x(:,k+1) = (kron(eye(n),A) + kron(eye(n),B*K1) + kron(L,B*K)) * x(:,k) ... 
             - (kron(eye(n),B*K1) + kron(L,B*K)) * h(:,k) ... 
             + kron(eye(n),B) * u_ff_accel ...                    
             + K3 * kron(eye(n),B) * F_Obstacles_Resultant ... 
             + K4 * kron(eye(n),B) * F_Agents_Resultant;       
end

% --- 绘图部分 ---
z = x' - h';
if size(z,1) > te_total +1 ; z = z(1:te_total+1,:); end
ep_agents = zeros(size(z,1), n);
for i = 1:n
    if size(z,2) >= 4*i
        ep_agents(:,i) = sqrt(sum(z(:, (4*(i-1)+1):4*i).^2, 2));
    end
end
if n > 0 && ~isempty(ep_agents)
    valid_ep_agents = ep_agents(:, any(ep_agents ~= 0, 1)); 
    if ~isempty(valid_ep_agents)
      [e_max_val,~] = max(valid_ep_agents,[],2);
      [e_min_val,~] = min(valid_ep_agents,[],2);
      error_formation = e_max_val - e_min_val;
    else
      error_formation = zeros(size(z,1),1);
    end
else
    error_formation = zeros(size(z,1),1);
end

figure(1)
clf
hold on
for obs_idx = 1:num_static_obstacles
    obs_center = p_obs_centers{obs_idx};
    rectangle('position', [obs_center(1)-obstacle_radius, obs_center(2)-obstacle_radius, 2*obstacle_radius, 2*obstacle_radius],'Curvature',1,'FaceColor','k','EdgeColor','k');
end

plot_colors = ['m'; 'g'; 'r'; 'b']; 
plot_markers_start = {'mo'; 'go'; 'ro'; 'bo'}; 
plot_markers_end = {'mP'; 'gP'; 'rP'; 'bP'};   
plot_line_styles_h = {'m--'; 'g--'; 'r--'; 'b--'}; 

final_t_idx = te_total + 1;
legend_handles = [];
legend_entries = {};

for i=1:n
    h_agent_traj = plot(x(4*i-3, 1:final_t_idx), x(4*i-1, 1:final_t_idx), [plot_colors(i),'.'],'markersize',3);
    plot(h(4*i-3, 1:final_t_idx), h(4*i-1, 1:final_t_idx), plot_line_styles_h{i},'LineWidth',1.0);
    plot(x(4*i-3, 1), x(4*i-1, 1), plot_markers_start{i},'LineWidth',2,'markersize',10, 'MarkerFaceColor', plot_colors(i));
    plot(x(4*i-3, final_t_idx), x(4*i-1, final_t_idx), plot_markers_end{i},'LineWidth',2,'markersize',10, 'MarkerFaceColor', plot_colors(i));
    legend_handles(end+1) = h_agent_traj;
    legend_entries{end+1} = ['Agent ', num2str(i)];
end

center_h_x = zeros(1, final_t_idx);
center_h_y = zeros(1, final_t_idx);
for k_plot = 1:final_t_idx
    temp_center_x = 0; temp_center_y = 0;
    for i_agent = 1:n 
        angle = 2*pi*(i_agent-1)/n + w_formation_rotation*t_vec_h(k_plot); 
        temp_center_x = temp_center_x + (h(4*i_agent-3, k_plot) - r_formation_agents * cos(angle));
        temp_center_y = temp_center_y + (h(4*i_agent-1, k_plot) - r_formation_agents * sin(angle));
    end
    center_h_x(k_plot) = temp_center_x/n;
    center_h_y(k_plot) = temp_center_y/n;
end
h_center_path = plot(center_h_x, center_h_y, 'k:', 'LineWidth', 1.5);
legend_handles(end+1) = h_center_path; 
legend_entries{end+1} = 'Desired Center Path';

axis equal; grid on; grid minor;
xlabel('X-axis/m'); ylabel('Y-axis/m');
if ~isempty(legend_handles)
    legend(legend_handles, legend_entries, 'Location', 'northeast');
end
title_str = 'Multi-Agent Formation Past Obstacle';
if Kv_obstacle > 0 || Kv_agent > 0 ; title_str = [title_str, ' (Vortex ON)']; else; title_str = [title_str, ' (Vortex OFF)']; end
title(title_str);
axis(axis_limits); % 使用场景定义的坐标轴范围

figure(2)
clf
hold on
time_axis_plot = (0:length(error_formation)-1)*l;
if length(time_axis_plot) > length(error_formation); time_axis_plot = time_axis_plot(1:length(error_formation));
elseif length(time_axis_plot) < length(error_formation); error_formation = error_formation(1:length(time_axis_plot));end
plot(time_axis_plot, error_formation,'r-','LineWidth',2)
grid on; grid minor;
xlabel('time(s)'); ylabel('Formation Error (Max-Min Norm Diff)');
title('Formation Error');
