% 起点 p1 和终点 p2 的坐标和方向
p1 = [5, -5, pi/4];  % 起点 [x, y, theta]
p2 = [5, 5, -pi/4];     % 终点 [x, y, theta]

% 最小转弯半径
r = 1;
h = 0;
% 步长（决定路径精细程度）
stepsize = 0.1;

% 是否安静模式，0 表示显示输出
quiet = 1;

% 调用 Dubins 曲线函数
path = dubins_curve_3D(p1, p2, r, stepsize, h, quiet);

% 显示路径结果
disp(path);

% 可视化路径
figure;
plot(path(:,1), path(:,2), 'b-', 'LineWidth', 2);
hold on;
scatter(p1(1), p1(2), 'r', 'filled');
scatter(p2(1), p2(2), 'g', 'filled');
text(p1(1), p1(2), ' Start', 'HorizontalAlignment', 'center');
text(p2(1), p2(2), ' End', 'HorizontalAlignment', 'center');
axis equal;
title('Dubins Path');
xlabel('X');
ylabel('Y');
grid on;
