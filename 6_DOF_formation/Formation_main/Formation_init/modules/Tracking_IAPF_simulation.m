function y = Tracking_IAPF_simulation(t, X, ctrl, all_obs)
    % 控制参数
    K_1 = ctrl.K_1;
    K_2 = ctrl.K_2;
    c_1 = ctrl.c_1; c_2 = ctrl.c_2;
    k_rep = ctrl.k_rep; k_rot = ctrl.k_rot; k_damp = ctrl.k_damp;
    r_safe = ctrl.r_safe;
    r = ctrl.r;
    v = ctrl.v;

    A = [0 1; 0 0]; B = [0; 1];
    N = 4;          % 智能体数
    y = zeros(4*N,1); % 输出维度20

    % ===========================
    % 编队目标轨迹 (例如四边形/菱形 + 平移)
    formation_ref = calcul_formation_h(v, r, t); % 16×1
    % disp(formation_ref);
    X_all = X(:);   % 16×1，当前所有agent状态

    % ===========================
    % 控制协议组成项
    % L为4阶环形拉普拉斯矩阵（图中结构）
    L = [ 2 -1  0 -1;
         -1  2 -1  0;
          0 -1  2 -1;
         -1  0 -1  2];

    U1 = - kron(L, kron(eye(2), K_1)) * X_all;
    U2 = - c_1 * kron(L, kron(eye(2), K_2)) * (X_all - formation_ref);

    surface = kron(L, kron(eye(2), K_2)) * (X_all - formation_ref);
    epsilon = 0.08;
    U4 = - c_2 * tanh(surface / epsilon);

    U_total = U1 + U2 + U4;

    % ===========================
    % APF 避障项
    U_apf = zeros(8,1); % 4 agents * 2维输入
    for i = 1:N
        idx = 4*i - 3; % X(i,:) = [x, vx, y, vy]
        pos = [X(idx), X(idx+2)]';
        vel = [X(idx+1), X(idx+3)]';
        F_rep = [0;0]; F_rot = [0;0];
        for j = 1:size(all_obs,1)
            obs = all_obs(j,:)';
            diff = pos - obs;
            dist = norm(diff);
            if dist < r_safe && dist > 1e-3
                F_rep = F_rep + k_rep * (1/dist - 1/r_safe) * (1/dist^3) * diff;
                rot_dir = cross([diff;0], [0;0;1]);
                F_rot = F_rot + k_rot * (rot_dir(1:2)/(dist^2));
            end
        end
        F_damp = -k_damp * vel;
        U_apf(2*i-1:2*i) = F_rep + F_rot + F_damp;
    end

    U_total = U_total + U_apf;

    % ===========================
    % 状态导数更新
    for i = 1:N
        idx = 4*i - 3;
        xi = X(idx:idx+1); yi = X(idx+2:idx+3);
        ui = U_total(2*i-1:2*i);
        y(idx:idx+1) = A*xi + B*ui(1);
        y(idx+2:idx+3) = A*yi + B*ui(2);
    end
end

