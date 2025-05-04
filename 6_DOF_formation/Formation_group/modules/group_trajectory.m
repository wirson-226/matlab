% function C = group_trajectory(t, num_groups)
%     % 每组的中心路径可自定义--绕圆时变
%     base = [10*cos(0.5*t), 10*sin(0.5*t)];
%     shift = 20;  % 每组间隔
%     C = zeros(num_groups,2);
%     for i = 1:num_groups
%         C(i,:) = base + [(i-1)*shift, 0];
%     end
% end


function C = group_trajectory(t, num_groups)
    % 每组的中心路径可自定义 -- 固定编队
    base = [10*cos(0.5*t), 10*sin(0.5*t)];
    shift = 20;  % 每组间隔
    C = zeros(num_groups,2);
    C(1,:) = [-10, -10];
    C(2,:) = [10, 10];

end


% function C = group_trajectory(t, num_groups)
%     % 每组的中心路径可自定义 -- 追踪编队
%     base = [10*cos(0.5*t), 10*sin(0.5*t)];
%     shift = 20;  % 每组间隔
%     C = zeros(num_groups,2);
%     C(2,:) = [t-10, t-10];
%     C(1,:) = [-10, -10];
% 
% end