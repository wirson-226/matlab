% modules/apf_avoidance.m
% 计算人工势场避障引导项（斥力矢量）
% 输入：
%   x_i: 智能体位置 (1x2)
%   obstacles: 所有障碍物位置 (Kx2)
%   eta: 避障增益系数
%   d0: 激活距离阈值（超过则不避障）
% 输出：
%   u_apf: 避障控制量 (1x2)

function u_apf = apf_avoidance(x_i, obstacles, eta, d0)
    u_apf = [0, 0];
    for k = 1:size(obstacles, 1)
        diff = x_i - obstacles(k,:);
        dist = norm(diff);
        if dist < d0 && dist > 1e-3  % 避免除以0
            grad = (diff / dist^3) * (1/dist - 1/d0);
            u_apf = u_apf + eta * grad;
        end
    end
end
