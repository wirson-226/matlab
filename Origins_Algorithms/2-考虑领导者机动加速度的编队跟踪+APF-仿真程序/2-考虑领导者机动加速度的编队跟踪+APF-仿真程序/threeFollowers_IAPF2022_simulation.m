function y = threeFollowers_IAPF2022_simulation(t, X)

% === 参数 ===
w = 0.314; r = 0.5;
w_lead = 0.157; r_lead = 0.3;
A = [0 1; 0 0]; B = [0; 1];

H = diag(0,2); H(1,1) = 2;
D = eye(3);
K_1 = [-1, 0];
K_2 = [-0.1251, -0.5732];
c_1 = 0.5; c_2 = 1;
L_H = [4  -1 -1;
       -1  1  0;
       -1  0  1];
N_1 = ones(3,1);
K1 = [-2, -1.2];
y = zeros(16,1);

% === Leader 控制 ===
U0 = kron(eye(2),K1)*(X(1:4)-calcul_singledrone_h(w_lead,r_lead,t)) + ...
     kron(eye(2),[0 1])*calcul_singledrone_hDer(w_lead,r_lead,t);
y(1:4) = kron(eye(2),A)*X(1:4) + kron(eye(2),B)*U0;

% === follower 编队控制 ===
U_foll = kron(eye(3), kron(eye(2), K_1))*(X(5:16) - kron(D*N_1, X(1:4))) ...
       + c_1*kron(L_H, kron(eye(2), K_2))*(X(5:16) - calcul_three_formation_h(w,r,t)) ...
       - c_1*kron(eye(3), kron(eye(2), K_2))*(kron(D*H*N_1, X(1:4))) ...
       + c_2*sign(kron(L_H, kron(eye(2), K_2))*(X(5:16) - calcul_three_formation_h(w,r,t)) ...
                  - kron(eye(3), kron(eye(2), K_2))*(kron(D*H*N_1, X(1:4))));

% === 改进 APF 避障项（论文方式） ===
% 参数设置
k_rep = 0.1;    % 斥力增益
k_rot = 0.2;    % 旋转力增益
k_damp = 0.5;   % 阻尼系数
r_safe = 0.2;   % 作用范围

% 障碍物设置
obstacles = [0,  -0.9;
             -1.5, 0.1;
              -1.5, 0.3]';

U_apf_total = zeros(6,1);  % 避障项（新）

for i = 1:3
    pos = [X(1 + 4 * i), X(3 + 4 * i)]';    % 位置
    vel = [X(2 + 4 * i), X(4 + 4 * i)]';    % 速度

    F_rep_total = [0; 0];
    F_rot_total = [0; 0];
    
%     disp("X: ");
%     disp(X);
%     disp("Pos: ");
%     disp(pos)
%     disp("Vel: ");
%     disp(vel);

    for j = 1:size(obstacles,2)
        obs = obstacles(:,j);
        diff = pos - obs;
        dist = norm(diff);
        
        if dist < r_safe && dist > 1e-3
            % === 斥力 ===（Eq.13）
            F_rep = k_rep * (1/dist - 1/r_safe) * (1/dist^3) * diff;
            F_rep_total = F_rep_total + F_rep;

            % === 旋转力 ===（Eq.15）
            e_z = [0; 0; 1];
            rot_dir = cross([diff; 0], e_z);  % 垂直于 diff 的方向（2D旋转）
            F_rot = k_rot * (rot_dir(1:2) / (dist^2));
            F_rot_total = F_rot_total + F_rot;
        end
    end

    % === 阻尼力 ===（Eq.17最后项）
    F_damp = -k_damp * vel;

    % 合力（论文 Eq.17: u_i = Fa + sum(F_r) + sum(F_e) + damping）
    U_apf = F_rep_total + F_rot_total + F_damp;

    % 存入
    U_apf_total(2*i-1:2*i) = U_apf;
end

% === 控制器最终合成 ===
U_foll = U_foll + U_apf_total;

% follower 动态更新
y(5:16) = kron(eye(6),A)*X(5:16) + kron(eye(6),B)*U_foll;

end
