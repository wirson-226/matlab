clc
clear
close all

% 系统参数 (来自您的代码)
n = 4;
l = 0.1;
K1 = kron(eye(2), [-2, -4]);
A = kron(eye(2), [1, l; 0, 1]);
B = kron(eye(2), [0; l]);
gamma = 0.95; % 未在后续使用，保留
theta = 0.95; % 未在后续使用，保留
P = dare(A + B * K1, B, eye(4), eye(2));
K = -inv(B' * P * B + eye(2)) * B' * P * (A + B * K1);
L = [1 0 0 -1;
    -1 1 0 0;
    0 -1 1 0;
    0 0 -1 1];

% APF 和涡旋场参数
K3 = 0.003; % 障碍物斥力（现在是涡旋斥力）增益
K4 = 0.003; % 智能体间斥力（现在是涡旋斥力）增益

% 新增：涡旋场增益系数 (来自论文思路, 需要仔细调试)
Kv_obstacle = 0; % 静态障碍物涡旋场增益
Kv_agent = 0;    % 智能体间涡旋场增益

te_total = 600; % *0.1s
t_total = 60;   % /s

% 仿真初始位置 (来自您的代码)
x0 = [2; 0; 0; 2; 0; 0.8; -2; 3; -2; 1.5; 0; 1; 3.5; -2.4; -2; 0.5];

% 圆形编队轨迹 (来自您的代码)
r_formation = 2; % 重命名避免与论文中其他r混淆
w_formation = 0.314;
t_vec = 0:l:t_total; % 时间向量
if length(t_vec) < te_total + 1
    t_corrected_end = (te_total) * l;
    t_vec = 0:l:t_corrected_end;
end
h = zeros(4 * n, length(t_vec));
for i = 1:n
    h(4*i-3 : 4*i, :) = [r_formation * cos(w_formation * t_vec + 2*pi*(i-1)/n);
                        -w_formation * r_formation * sin(w_formation * t_vec + 2*pi*(i-1)/n);
                         r_formation * sin(w_formation * t_vec + 2*pi*(i-1)/n);
                         w_formation * r_formation * cos(w_formation * t_vec + 2*pi*(i-1)/n)];
end

% 障碍物数据 (来自您的代码)
[xo, yo, zo_cylinder] = cylinder(0.25, 50); % zo_cylinder 未在2D中使用
p_obs_centers = { [1.5; 1.5], [-1.5; -1.5], [-1.3; 2.5] }; % 障碍物中心点 (2x1向量)
num_static_obstacles = length(p_obs_centers);
d_safe = 0.8; % 障碍物斥力影响距离
d0 = 1;     % 智能体间斥力影响距离 (论文中是 ρ0)
b = 4;      % 斥力函数参数

% 初始化状态和力矩阵
x = zeros(4*n, te_total + 1);
x(:, 1) = x0;

% 主仿真循环
for k = 1:te_total
    
    % 初始化当前时间步的总涡旋障碍物力和总涡旋智能体间力
    F_Vortex_Obstacles_Total = zeros(2*n, 1);
    F_Vortex_Agents_Total = zeros(2*n, 1);

    % --- 步骤 1: 为静态障碍物计算涡旋场力 ---
    for i = 1:n % 遍历每个智能体
        agent_pos_i = [x(4*i-3, k); x(4*i-1, k)];       % 智能体 i 的当前位置
        target_pos_i = [h(4*i-3, k); h(4*i-1, k)];     % 智能体 i 的当前目标位置 (来自h轨迹)
        
        AF_vec_i = target_pos_i - agent_pos_i; % 智能体i 指向其目标F的向量 AF

        current_agent_total_vortex_obs_force_x = 0;
        current_agent_total_vortex_obs_force_y = 0;

        for obs_idx = 1:num_static_obstacles % 遍历每个静态障碍物
            obs_center_j = p_obs_centers{obs_idx};

            % 计算智能体i与障碍物obs_idx表面最近点的距离和原始斥力
            % (这部分逻辑基于您原始代码对F_obs1,2,3的计算方式)
            dist_to_surf_points = sqrt(((agent_pos_i(1) - (obs_center_j(1) + xo(1,:))).^2 + ...
                                        (agent_pos_i(2) - (obs_center_j(2) + yo(1,:))).^2));
            [min_dist_ij, nearest_surf_idx] = min(dist_to_surf_points);
            
            Fr_x_orig_ij = 0; % 智能体i 相对于障碍物j 的原始x方向斥力
            Fr_y_orig_ij = 0; % 智能体i 相对于障碍物j 的原始y方向斥力

            if min_dist_ij < d_safe && min_dist_ij > 1e-3 % 避免除以零
                % 原始斥力大小 (来自您的代码)
                rep_magnitude = (b / min_dist_ij - 1 / d_safe)^2 * (b / (min_dist_ij^2));
                
                % 障碍物表面最近点
                obs_surface_point_j = [obs_center_j(1) + xo(1, nearest_surf_idx);
                                       obs_center_j(2) + yo(1, nearest_surf_idx)];
                
                % 原始斥力方向 (从障碍物指向智能体)
                dir_vec_obs_to_agent_ij = (agent_pos_i - obs_surface_point_j) / min_dist_ij;
                
                Fr_x_orig_ij = rep_magnitude * dir_vec_obs_to_agent_ij(1);
                Fr_y_orig_ij = rep_magnitude * dir_vec_obs_to_agent_ij(2);

                % --- 计算涡旋场 ---
                AO_vec_ij = obs_surface_point_j - agent_pos_i; % 智能体i 指向障碍物表面点j 的向量 AO

                % 自旋向量R的Z分量 (Rz = AFx * AOy - AFy * AOx)
                Rz_ij = AF_vec_i(1) * AO_vec_ij(2) - AF_vec_i(2) * AO_vec_ij(1);

                % 根据Rz符号应用涡旋变换 (论文Table 2的2D适配)
                if Rz_ij <= 0
                    Fv_x_ij = Fr_x_orig_ij + Kv_obstacle * Fr_y_orig_ij;
                    Fv_y_ij = Fr_y_orig_ij - Kv_obstacle * Fr_x_orig_ij;
                else % Rz_ij > 0
                    Fv_x_ij = Fr_x_orig_ij - Kv_obstacle * Fr_y_orig_ij;
                    Fv_y_ij = Fr_y_orig_ij + Kv_obstacle * Fr_x_orig_ij;
                end
                current_agent_total_vortex_obs_force_x = current_agent_total_vortex_obs_force_x + Fv_x_ij;
                current_agent_total_vortex_obs_force_y = current_agent_total_vortex_obs_force_y + Fv_y_ij;
            end
        end
        F_Vortex_Obstacles_Total(2*i-1, 1) = current_agent_total_vortex_obs_force_x;
        F_Vortex_Obstacles_Total(2*i, 1)   = current_agent_total_vortex_obs_force_y;
    end

    % --- 步骤 2: 为智能体间避碰计算涡旋场力 ---
    for i = 1:n % 对于智能体 i
        agent_pos_i = [x(4*i-3, k); x(4*i-1, k)];
        target_pos_i = [h(4*i-3, k); h(4*i-1, k)]; % 智能体i的当前目标
        AF_vec_i = target_pos_i - agent_pos_i;

        current_agent_total_vortex_agent_force_x = 0;
        current_agent_total_vortex_agent_force_y = 0;

        for j = 1:n % 考虑其他智能体 j
            if i == j
                continue; % 不计算对自身的力
            end
            agent_pos_j = [x(4*j-3, k); x(4*j-1, k)]; % 其他智能体j的位置 (作为动态障碍物O)

            % 计算智能体i与智能体j的原始斥力
            dist_ij_vec = agent_pos_i - agent_pos_j;
            dist_ij = norm(dist_ij_vec);

            Fr_x_orig_agent_ij = 0;
            Fr_y_orig_agent_ij = 0;

            if dist_ij < d0 && dist_ij > 1e-3 % 避免除以零
                % 您的原始智能体间斥力函数 (指数形式)
                % (b/exp(d_ij)-1/exp(d0))^2*(b/(exp(d_ij)))*[(x(4*i-3,k)-x(4*j-3,k))/d_ij(i,j);(x(4*i-1,k)-x(4*j-1,k))/d_ij(i,j)];
                rep_magnitude_agent = (b / exp(dist_ij) - 1 / exp(d0))^2 * (b / exp(dist_ij));
                dir_vec_j_to_i = dist_ij_vec / dist_ij; % 从j指向i的单位向量 (斥力方向)

                Fr_x_orig_agent_ij = rep_magnitude_agent * dir_vec_j_to_i(1);
                Fr_y_orig_agent_ij = rep_magnitude_agent * dir_vec_j_to_i(2);

                % --- 计算涡旋场 ---
                AO_vec_agent_ij = agent_pos_j - agent_pos_i; % 智能体i 指向 智能体j(障碍物)的向量 AO

                % 自旋向量R的Z分量
                Rz_agent_ij = AF_vec_i(1) * AO_vec_agent_ij(2) - AF_vec_i(2) * AO_vec_agent_ij(1);
                
                % 根据Rz符号应用涡旋变换
                if Rz_agent_ij <= 0
                    Fv_x_agent_ij = Fr_x_orig_agent_ij + Kv_agent * Fr_y_orig_agent_ij;
                    Fv_y_agent_ij = Fr_y_orig_agent_ij - Kv_agent * Fr_x_orig_agent_ij;
                else % Rz_agent_ij > 0
                    Fv_x_agent_ij = Fr_x_orig_agent_ij - Kv_agent * Fr_y_orig_agent_ij;
                    Fv_y_agent_ij = Fr_y_orig_agent_ij + Kv_agent * Fr_x_orig_agent_ij;
                end
                current_agent_total_vortex_agent_force_x = current_agent_total_vortex_agent_force_x + Fv_x_agent_ij;
                current_agent_total_vortex_agent_force_y = current_agent_total_vortex_agent_force_y + Fv_y_agent_ij;
            end
        end
        F_Vortex_Agents_Total(2*i-1, 1) = current_agent_total_vortex_agent_force_x;
        F_Vortex_Agents_Total(2*i, 1)   = current_agent_total_vortex_agent_force_y;
    end

        % 为前馈计算期望的加速度
    u_ff_accel = zeros(2*n, 1); % 初始化一个 2n x 1 的列向量
    for agent_idx = 1:n
        % 假设 h 的第 (4*agent_idx-2) 行是智能体 agent_idx 的 x 方向速度
        % 假设 h 的第 (4*agent_idx)   行是智能体 agent_idx 的 y 方向速度
    
        % 检查 h 是否有足够的列来安全访问 h(:, k+1)
        if k+1 <= size(h, 2)
            desired_vel_x_k = h(4*agent_idx-2, k);
            desired_vel_x_k_plus_1 = h(4*agent_idx-2, k+1);
            desired_accel_x = (desired_vel_x_k_plus_1 - desired_vel_x_k) / l;
    
            desired_vel_y_k = h(4*agent_idx, k);
            desired_vel_y_k_plus_1 = h(4*agent_idx, k+1);
            desired_accel_y = (desired_vel_y_k_plus_1 - desired_vel_y_k) / l;
        else
            % 如果 k+1 超出 h 的范围 (例如在最后一个时间步 te_total)
            % 可以假设加速度为0，或者根据您的具体逻辑处理
            desired_accel_x = 0;
            desired_accel_y = 0;
        end
    
        u_ff_accel(2*agent_idx-1) = desired_accel_x; % 智能体 agent_idx 的 x 方向期望加速度
        u_ff_accel(2*agent_idx)   = desired_accel_y; % 智能体 agent_idx 的 y 方向期望加速度
    end

    % --- 状态更新 ---
    % 注意: F_Vortex_Obstacles_Total 和 F_Vortex_Agents_Total 已经是 2n x 1 的“广义力”向量
    % 它们可以直接与 B 矩阵（通过 Kronecker 积）相乘
   % --- 状态更新方程 ---
    x(:,k+1) = (kron(eye(n),A+B*K1) + kron(L,B*K)) * x(:,k) ...  % 当前状态和邻居状态影响
             - kron(eye(n),B*K1) * h(:,k) ...                     % 对参考状态h的反馈
             + kron(L,B*K) * (x(:,k)-h(:,k)) ...                  % 编队误差反馈
             + kron(eye(n),B) * u_ff_accel ...                    % <<--- 使用修正后的前馈项
             + K3 * kron(eye(n),B) * F_Vortex_Obstacles_Total ... % 障碍物涡旋斥力
             + K4 * kron(eye(n),B) * F_Vortex_Agents_Total;       % 智能体间涡旋斥力

end

% --- 绘图部分 (与您的原始代码一致或进行相应修改) ---
% Formation error calculation
z=x'-h';
if size(z,1) > te_total +1
    z = z(1:te_total+1,:);
end
z1=zeros(size(z,1),4); z2=zeros(size(z,1),4); z3=zeros(size(z,1),4); z4=zeros(size(z,1),4);
if size(z,2) >= 4; z1=z(:,1:4); end
if size(z,2) >= 8; z2=z(:,5:8); end
if size(z,2) >= 12; z3=z(:,9:12); end
if size(z,2) >= 16; z4=z(:,13:16); end
ep1=sum(z1,2); ep2=sum(z2,2); ep3=sum(z3,2); ep4=sum(z4,2);
ep=[ep1,ep2,ep3,ep4];
e_max=zeros(size(ep,1),1); e_min=zeros(size(ep,1),1);
if ~isempty(ep)
    [e_max,~]=max(ep,[],2);
    [e_min,~]=min(ep,[],2);
end
error_formation=e_max-e_min;

% Figure 1: Trajectories
figure(1)
hold on
rectangle('position', [p_obs_centers{1}(1)-0.25, p_obs_centers{1}(2)-0.25, 0.5, 0.5],'Curvature',1,'FaceColor','k');
rectangle('position', [p_obs_centers{2}(1)-0.25, p_obs_centers{2}(2)-0.25, 0.5, 0.5],'Curvature',1,'FaceColor','k');
rectangle('position', [p_obs_centers{3}(1)-0.25, p_obs_centers{3}(2)-0.25, 0.5, 0.5],'Curvature',1,'FaceColor','k');

colors = ['m.';
    'g.';
    'r.';
    'b.'];
colors_P = ['mP'; 'gP'; 'rP'; 'bP']; % For pentagram marker
colors_o = ['mo'; 'go'; 'ro'; 'bo']; % For circle marker
final_t_idx = te_total + 1;

for i=1:n
    plot(x(4*i-3, 1:final_t_idx), x(4*i-1, 1:final_t_idx), colors(i,:),'markersize',3); % Plotting positions (x,y)
    plot(x(4*i-3, 1), x(4*i-1, 1), colors_o(i,:),'LineWidth',2,'markersize',10); % Start
    plot(x(4*i-3, final_t_idx), x(4*i-1, final_t_idx), colors_P(i,:),'LineWidth',2,'markersize',10); % End
    if size(h,2) >= final_t_idx
      plot(h(4*i-3, 1:final_t_idx), h(4*i-1, 1:final_t_idx), [colors(i,1),'--'],'LineWidth',1.0); % Desired path
    end
end

% Plotting final formation shape line (connecting agents at the end)
final_pos_x = zeros(n+1,1); final_pos_y = zeros(n+1,1);
for i=1:n
    final_pos_x(i) = x(4*i-3, final_t_idx);
    final_pos_y(i) = x(4*i-1, final_t_idx);
end
final_pos_x(n+1) = x(1, final_t_idx); % Connect last to first
final_pos_y(n+1) = x(3, final_t_idx);
plot(final_pos_x, final_pos_y, 'k--');

axis equal; grid on; grid minor;
xlabel('X-axis/m'); ylabel('Y-axis/m');
legend_entries = cell(1,n);
for i=1:n; legend_entries{i} = ['Agent ', num2str(i)]; end
legend(legend_entries(1:n)); % Only legend for agent trajectories for clarity
title('Multi-Agent Trajectories with Vortex APF');

% Figure 3: Formation Error (from your code)
figure(3)
hold on
time_axis_error = (0:length(error_formation)-1)*l;
if length(time_axis_error) > length(error_formation)
    time_axis_error = time_axis_error(1:length(error_formation));
end
plot(time_axis_error, error_formation(:,1),'r-','LineWidth',2)
grid on; grid minor;
xlabel('time(s)'); ylabel('formation error');
title('Formation Error');