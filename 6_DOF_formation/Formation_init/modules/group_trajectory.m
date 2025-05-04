% function C = group_trajectory(t, num_groups)
%     % 每组的中心路径可自定义--绕圆时变
%     base = [10*cos(0.5*t), 10*sin(0.5*t)];
%     shift = 20;  % 每组间隔
%     C = zeros(num_groups,2);
%     for i = 1:num_groups
%         C(i,:) = base + [(i-1)*shift, 0];
%     end
% end


% function C = group_trajectory(t, num_groups)
%     % 每组的中心路径可自定义 -- 固定编队
%     base = [10*cos(0.5*t), 10*sin(0.5*t)];
%     shift = 20;  % 每组间隔
%     C = zeros(num_groups,2);
%     C(1,:) = [-10, -10];
%     C(2,:) = [10, 10];
% 
% end


function C = group_trajectory(t, num_groups, dt)
    % % 每组的中心路径可自定义 -- 追踪编队
    C = zeros(num_groups,2);
    persistent center1 v_center1
    if isempty(center1), center1 = [10, 10]; end
    if isempty(v_center1), v_center1 = [0, 0]; end
    C(2,:) = [t-10, t-10];
% 使用一阶追踪 + 平滑靠近，保留包围感
      k_p = 5; k_d = 0.3;
      e = C(2,:) - center1;
      center1_dot = k_p * e - k_d * v_center1;
      v_center1 = center1_dot;
      center1 = center1 + center1_dot * dt;
      C(1,:) = center1 ;

end