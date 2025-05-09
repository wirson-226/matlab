% function [targets, group_centers] = generate_targets_grouped(t, group_ids, x, mode, all_obs, r_obs,dt)
%     N = length(group_ids);
%     targets = zeros(N, 2);
%     groups = unique(group_ids);
% 
%     % 每组分别生成一个中心
%     group_centers = group_trajectory(t, length(groups),dt);  % <--- 保留作为第二个输出
% 
%     for g = 1:length(groups)
%         idx = find(group_ids == groups(g));
%         n_group = length(idx);
%         alpha = 1 + 0.2 * sin(0.1*t);
%         r = 10; % 半径
%         if strcmp(mode, 'rotating')
%             omega = 50;
%             R = [cos(omega*t), -sin(omega*t); sin(omega*t), cos(omega*t)];
%         else
%             R = eye(2);
%         end
%         center = group_centers(g,:);
%         for k = 1:n_group
%             ang = 2*pi*(k-1)/n_group;
%             p = alpha * r * [cos(ang), sin(ang)];
%             targets(idx(k), :) = center + (R * p')';
%         end
%     end
% 
%     % 避障修正
%     offset = path_planner_avoidance(targets, all_obs, r_obs);
%     targets = targets + offset;
% end

function [targets, group_centers] = generate_targets_grouped(t, group_ids, x, mode, all_obs, r_obs, dt)
    % 输入参数说明：
    %   group_ids: 智能体分组标识（例如 [1,1,1,1,1,2,2,2] 表示前5个为组1，后3个为组2）
    %   新增组参数通过结构体传递（见下方代码）
    
    N = length(group_ids);
    targets = zeros(N, 2);
    groups = unique(group_ids);
    
    % ===== 组参数配置 =====
    group_params = struct();
    group_params(1).radius = 15;      % 组1（五边形）半径
    group_params(1).omega = 0.5;     % 组1角速度（rad/s）
    group_params(1).phase = 0;       % 组1初始相位偏移
    group_params(2).radius = 4;      % 组2（三角形）半径 
    group_params(2).omega = -0.3;    % 组2角速度（反向旋转）
    group_params(2).phase = pi/2;    % 组2初始相位偏移
    % =====================
    
    % 生成各组中心轨迹
    group_centers = group_trajectory(t, length(groups), dt);  % 假设该函数返回每组中心坐标
    
    for g = 1:length(groups)
        idx = find(group_ids == groups(g));
        n_group = length(idx);
        center = group_centers(g,:);
        
        % 获取当前组参数
        r = group_params(g).radius;
        omega = group_params(g).omega;
        phase = group_params(g).phase;
        
        % 生成绕中心均匀分布的目标点
        for k = 1:n_group
            % 计算当前角度（含旋转和初始相位）
            ang = 2*pi*(k-1)/n_group + omega*t + phase;
            
            % 五边形/三角形顶点坐标
            p = r * [cos(ang), sin(ang)];
            
            % 旋转模式处理
            if strcmp(mode, 'rotating')
                R = [cos(omega*t), -sin(omega*t); sin(omega*t), cos(omega*t)];
                targets(idx(k), :) = center + (R * p')';
            else
                targets(idx(k), :) = center + p;
            end
        end
    end
    
    % 避障修正
    offset = path_planner_avoidance(targets, all_obs, r_obs);
    targets = targets + offset;
end




