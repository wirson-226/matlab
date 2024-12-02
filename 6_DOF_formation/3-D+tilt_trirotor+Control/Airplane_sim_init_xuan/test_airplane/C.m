% 首先，假设你已经下载并添加了dubins路径库到MATLAB路径中
% 这里我们使用dubins库生成路径

% 定义多个点 [x, y, theta]
% 假设起始点和目标点的格式为 [x, y, theta]
points = [-5, -5, pi/4;    % 起点
          -5, 5, pi/2;     % 中间点
          5, 5, pi;        % 中间点
          5, -5, -pi/2];   % 终点

% 最小转弯半径
radius = 1;

% 初始化绘图
figure;
hold on;
axis equal;
grid on;
xlabel('X');
ylabel('Y');
title('Dubins Path for Multiple Points');

% 绘制每段路径
for i = 1:size(points, 1) - 1
    % 当前点与下一个点
    start_point = points(i, :);  % [x, y, theta]
    end_point = points(i + 1, :); % [x, y, theta]
    
    % 计算Dubins路径
    % dubins路径函数返回一个路径结构，包含路径类型和轨迹点
    path = dubins_path(start_point, end_point, radius);
    
    % 提取轨迹点
    [path_points, ~] = path.sample(100); % 采样100个点，增加平滑度
    
    % 绘制路径
    plot(path_points(:,1), path_points(:,2), 'b-', 'LineWidth', 2);
end

% 绘制所有点
plot(points(:,1), points(:,2), 'ro', 'MarkerFaceColor','r');
