function y = optimizedFormationControl(t, X, ctrl, all_obs)
    % 参数提取
    w = ctrl.w; r = ctrl.r;
    w_lead = ctrl.w_lead; r_lead = ctrl.r_lead;
    K1 = ctrl.K1;
    K_1 = ctrl.K_1; K_2 = ctrl.K_2;
    c_1 = ctrl.c_1; c_2 = ctrl.c_2;
    k_rep = ctrl.k_rep; k_rot = ctrl.k_rot; 
    k_damp = ctrl.k_damp; r_safe = ctrl.r_safe;
    obs_radius = ctrl.obs_radius; % 新增障碍物半径参数
    

    % 系统矩阵
    A = [0 1; 0 0]; B = [0; 1];
    H = zeros(3,3); H(1,1) = 2;  % H必须是3×3矩阵
    D = eye(3);
    L_H = [4 -1 -1; -1 1 0; -1 0 1];
    N_1 = ones(3,1);
    
    y = zeros(16,1);
    
    % === Leader控制 ===
    leader_ref = calcul_singledrone_h(w_lead, r_lead, t);
    leader_ref_dot = calcul_singledrone_hDer(w_lead, r_lead, t);
    
    U0 = kron(eye(2),K1)*(X(1:4) - leader_ref) + ...
         kron(eye(2),[0 1])*leader_ref_dot;
    y(1:4) = kron(eye(2),A)*X(1:4) + kron(eye(2),B)*U0;
    
    % === Follower编队控制 ===
    formation_ref = calcul_three_formation_h(w, r, t);
    
    
    % 分步计算以确保维度正确
    X_followers = X(5:16);  % 12×1
    
    % 第一项：跟随项
    U_term1 = kron(eye(3), kron(eye(2), K_1)) * (X_followers - kron(D*N_1, X(1:4)));
    
    % 第二项：编队项
    U_term2 = c_1 * kron(L_H, kron(eye(2), K_2)) * (X_followers - formation_ref);
    
    % 第三项：修正项
    U_term3 = c_1 * kron(eye(3), kron(eye(2), K_2)) * kron(D*H*N_1, X(1:4));
    
    % 第四项：滑模项
    sliding_surface = kron(L_H, kron(eye(2), K_2)) * (X_followers - formation_ref) - ...
                     kron(eye(3), kron(eye(2), K_2)) * kron(D*H*N_1, X(1:4));
    % U_term4 = c_2 * sign(sliding_surface);
    % 效果
    % < 0.05	抖动大、接近 sign()
    % 0.05–0.15	平衡控制强度与平滑性
    % > 0.2	抑制抖动强，控制滞后
    epsilon = 0.08;  % 可调边界层参数，越小越接近 sign()，但越容易抖动ε值	
    U_term4 = c_2 * tanh(sliding_surface / epsilon);

    
    % 合成控制
    U_foll = U_term1 + U_term2 - U_term3 + U_term4;
    
    
    % ===== 3. 增强型APF避障 =====
    U_apf = zeros(6,1);
    for i = 1:3
        idx = 4*i + [1,3]; % 提取xy位置索引
        pos = X(idx)';
        vel = X(idx+1)';
        
        % 合并静态和动态障碍物
        [F_rep, F_rot] = deal([0,0]);
        for j = 1:size(all_obs.static,1)
            [f_rep, f_rot] = apf_force(pos, all_obs.static(j,:), vel, ...
                                 k_rep, k_rot, r_safe, obs_radius);
            F_rep = F_rep + f_rep;
            F_rot = F_rot + f_rot;
        end
        
        % 动态障碍物处理 (需考虑相对速度)
        if isfield(all_obs, 'moving')
            for j = 1:size(all_obs.moving.pos,1)
                obs_pos = all_obs.moving.pos(j,:) + t*all_obs.moving.vel(j,:);
                [f_rep, f_rot] = apf_force(pos, obs_pos, vel, ...
                                     k_rep*1.5, k_rot*1.2, r_safe*1.2, obs_radius);
                F_rep = F_rep + f_rep;
                F_rot = F_rot + f_rot;
            end
        end
        
        % 阻尼项和合成控制
        F_damp = -k_damp * vel;
        U_apf(2*i-1:2*i) = F_rep + F_rot + F_damp;
    end
    
    % ===== 4. 最终控制输出 =====

    U_foll = U_foll + U_apf;
    
    y(5:16) = kron(eye(6),A)*X(5:16) + kron(eye(6),B)*U_foll;
end

%% APF力计算子函数
function [F_rep, F_rot] = apf_force(pos, obs_pos, vel, k_rep, k_rot, r_safe, obs_radius)
    diff = pos - obs_pos;
    dist = norm(diff) - obs_radius; % 考虑障碍物半径
    
    F_rep = [0,0];
    F_rot = [0,0];
    
    if dist < r_safe && dist > 0.01
        % 改进的排斥力模型
        rep_gain = k_rep * (1/dist - 1/r_safe) / dist^2;
        F_rep = rep_gain * diff/norm(diff);
        
        % 自适应旋绕力
        rot_dir = [diff(2), -diff(1)]; % 垂直向量
        F_rot = k_rot * rot_dir / (dist^1.5);
    end
end