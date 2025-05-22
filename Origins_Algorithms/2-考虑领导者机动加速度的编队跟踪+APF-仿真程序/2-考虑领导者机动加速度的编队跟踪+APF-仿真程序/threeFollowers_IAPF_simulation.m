function y = threeFollowers_IAPF_simulation(t, X)

% 控制参数
w = 0.314; r = 0.5;
w_lead = 0.157; r_lead = 0.3;

A = [0 1; 0 0];
B = [0; 1];

H = diag(0,2); H(1,1) = 2;
D = eye(3);  % follower-leader通信关系

K_1 = [-1, 0];
K_2 = [-0.1251, -0.5732];
c_1 = 0.5;
c_2 = 1;

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

% === 编队控制项 ===
U_foll = kron(eye(3), kron(eye(2), K_1))*(X(5:16) - kron(D*N_1, X(1:4))) ...
       + c_1*kron(L_H, kron(eye(2), K_2))*(X(5:16) - calcul_three_formation_h(w,r,t)) ...
       - c_1*kron(eye(3), kron(eye(2), K_2))*(kron(D*H*N_1, X(1:4))) ...
       + c_2*sign(kron(L_H, kron(eye(2), K_2))*(X(5:16) - calcul_three_formation_h(w,r,t)) ...
                  - kron(eye(3), kron(eye(2), K_2))*(kron(D*H*N_1, X(1:4))));

%% 改进人工势场法
Kmpf = 1;      % 势场增益
KFl = 0.5;     % 轨迹吸引力增益
Kalert = 0.75; % 避障启动系数
r = 0.2;       % 影响半径
Robs = 0.3;    % 障碍物物理半径
vt = [X(2); X(4)];   % 目标航向方向

% 三个障碍物
obstacles = [0.5,  0.5;
             -1.5, 0.2;
              1.5, -1.0]';

% follower 状态提取
follower_pos = [X(5), X(9), X(13);  % x
                X(7), X(11), X(15)];% y
U_rep_total = zeros(6,1);

% === Improved APF 避障力计算 ===
for i = 1:3
    pos = follower_pos(:,i);
    vel = X(6*(i-1)+2 : 6*(i-1)+2+1);  % 当前速度 vx, vy
    F_total = [0; 0];
    
    for j = 1:size(obstacles,2)
        p_obs = obstacles(:,j);
        dic = p_obs - pos;
        dist = norm(dic);
        
        if dist < Kalert * r
            % 计算变形参数
            vi = vel;
            dsv = 0.1;
            Ri_min = norm(vi)^2 / (9.8 * tan(pi/8));
            ds = 0.8 * Ri_min;
            S = -vi / norm(vi) * dsv;
            dic_s = p_obs - pos + S;
            
            gvi = acos(dot(dic_s, vi) / (norm(dic_s)*norm(vi)));
            if gvi < pi/2
                C0 = (r/(r + Ri_min - ds))^2;
            else
                C0 = (r/(r + ds))^2;
            end
            C = (1 - C0)*sin(gvi)^2 + C0;
            
            Jmpf = Kmpf * exp(-C * (norm(dic_s)/r)^2);
            F_obs = -Jmpf * 2/(r^2) * dic_s;
            F_total = F_total + F_obs;
        end
    end
    
    % 吸引力轨迹项（简单模型）
    Dn = (pi/2) * atan(2 * dot(vt, pos)) / 10;
    F_traj = vt + Dn;
    F_total = F_total + KFl * F_traj;
    
    U_rep_total(2*i-1:2*i) = F_total;
end

% disp("Pos:");
% disp([X(5), X(9), X(13);  % x
%                 X(7), X(11), X(15)]);
% disp("Vel:");
% disp([X(6), X(10), X(14);  % x
%                 X(8), X(12), X(16)]);
% disp(", U_foll:");
% disp(U_foll);
% disp(", U_rep_total:");
% disp(U_rep_total);

% 合并控制
U_foll = U_foll + U_rep_total;

% follower 动态更新
y(5:16) = kron(eye(6), A)*X(5:16) + kron(eye(6), B)*U_foll;

end
