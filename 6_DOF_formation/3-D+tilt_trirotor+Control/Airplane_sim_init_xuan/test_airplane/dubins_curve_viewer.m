% 点集合，每一行为一个点，包含 x, y, theta
points = [
    5, -5, pi/4;
    10, 0, pi/3;
    15, 5, -pi/4;
    10, 10, pi/6;
    5, 15, 0
];

% 最小转弯半径
r = 10;
h = 0; % 高度（平面内保持不变）
% 步长（决定路径精细程度）
stepsize = 0.1;
% 是否安静模式，0 表示显示输出
quiet = 1;
% 调用生成路径函数
full_path = generate_dubins_path(points, r, stepsize, h, quiet);
% 提取路径的 x, y 坐标
traj_xy = [full_path(:,1), full_path(:,2)];






% 可视化路径
figure;
plot(traj_xy(:,1), traj_xy(:,2), 'b-', 'LineWidth', 2);
hold on;

% 绘制所有点
scatter(points(:,1), points(:,2), 50, 'r', 'filled');
text(points(:,1), points(:,2), {'P1', 'P2', 'P3', 'P4', 'P5'}, 'VerticalAlignment', 'bottom');

% 配置图形属性
axis equal;
title('Continuous Dubins Path');
xlabel('X');
ylabel('Y');
grid on;

hold off;
