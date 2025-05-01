% modules/path_planner_avoidance.m
% 将靠近障碍物的参考点投影到安全边界圆环上，保持编队结构
% 输入：
%   raw_targets: 原始参考点 (Nx2)
%   obstacles: 所有障碍物位置 (Kx2)
% 输出：
%   offset: 参考点的修正量 (Nx2)
function offset = path_planner_avoidance(raw_targets, obstacles,r_obs)
    r_safe = 1.0;
    % r_obs = 1.5;
    eps = 0.2;  % 安全冗余边距
    r_thresh = r_safe + r_obs + eps;  % 投影生效半径

    N = size(raw_targets,1);
    offset = zeros(N,2);

    for i = 1:N
        total_repel = [0, 0];  % 初始化合成斥力
        for k = 1:size(obstacles,1)
            diff = raw_targets(i,:) - obstacles(k,:);
            dist = norm(diff);
            if dist < r_thresh && dist > 1e-3
                gain = exp(-(dist / r_thresh)^2);
                projected = obstacles(k,:) + (diff / dist) * r_thresh;
                total_repel = total_repel + gain * (projected - raw_targets(i,:));
            end
        end
        offset(i,:) = offset(i,:) + total_repel;
    end

    % ==== 增加参考点之间的防撞逻辑 ====
    min_spacing = 1.5;  % 最小安全间隔
    for i = 1:N
        for j = i+1:N
            diff = raw_targets(i,:) - raw_targets(j,:);
            dist = norm(diff);
            if dist < min_spacing && dist > 1e-3
                repel = 0.5 * (min_spacing - dist) * (diff / dist);  % 平均作用力偏移
                offset(i,:) = offset(i,:) + repel;
                offset(j,:) = offset(j,:) - repel;
            end
        end
    end
end

