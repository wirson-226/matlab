function y = threeFollowers_formation_tracking_simulation(t,X)

% 参数设置
w = 0.314;
r = 0.5;

w_lead = 0.157;
r_lead = 0.3;

A = [0 1; 0 0];
B = [0; 1];

H = diag(0,2); H(1,1) = 2;
D = diag(0,2); D(1:3,1:3) = eye(3);

K_1 = [-1, 0];
K_2 = [-0.1251, -0.5732];
c_1 = 0.5;
c_2 = 1;

L_H = [4  -1 -1;
       -1  1  0;
       -1  0  1];

N_1 = ones(3,1);
y = zeros(16,1);
K1 = [-2, -1.2];

% === 控制输入 ===
U0 = kron(eye(2),K1)*(X(1:4)-calcul_singledrone_h(w_lead,r_lead,t)) + ...
     kron(eye(2),[0 1])*calcul_singledrone_hDer(w_lead,r_lead,t);

y(1:4) = kron(eye(2),A)*X(1:4) + kron(eye(2),B)*U0;

% === follower控制输入 ===
U_foll = kron(eye(3), kron(eye(2), K_1))*(X(5:16) - kron(D*N_1, X(1:4))) ...
       + c_1*kron(L_H, kron(eye(2), K_2))*(X(5:16) - calcul_three_formation_h(w,r,t)) ...
       - c_1*kron(eye(3), kron(eye(2), K_2))*(kron(D*H*N_1, X(1:4))) ...
       + c_2*sign(kron(L_H, kron(eye(2), K_2))*(X(5:16) - calcul_three_formation_h(w,r,t)) ...
                  - kron(eye(3), kron(eye(2), K_2))*(kron(D*H*N_1, X(1:4))));

% === 人工势场法避障项 ===
% 三个障碍物（其中一个在路径上）
obstacles = [0.5,  0.5;
             -1.5, 0.1;
              -1.5, 0.3]';
d_safe = 0.2;
k_rep = 0.3;

U_rep_total = zeros(6,1); % 三个follower

% follower位置提取
follower_pos = [X(5), X(9), X(13);
                X(7), X(11), X(15)];

for i = 1:3
    pos = follower_pos(:,i);
    F_rep = zeros(2,1);
    for j = 1:size(obstacles,2)
        obs = obstacles(:,j);
        diff = pos - obs;
        dist = norm(diff);
        if dist < d_safe && dist > 1e-3
            F_rep = F_rep + k_rep * (1/dist - 1/d_safe) * (1/dist^3) * diff;
        end
    end
    U_rep_total(2*i-1:2*i) = F_rep;
end

% 最终控制输入加避障项
U_foll = U_foll + U_rep_total;

% follower状态更新
y(5:16) = kron(eye(6), A) * X(5:16) + kron(eye(6), B) * U_foll;

end
