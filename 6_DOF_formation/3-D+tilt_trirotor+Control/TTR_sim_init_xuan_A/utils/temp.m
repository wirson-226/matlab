% 生成 3D 数据
x = linspace(-10, 10, 100);
y = linspace(-10, 10, 100);
z = sin(x) + cos(y);

% 绘制 3D 图形
figure;
plot3(x, y, z);

% 调整 y 轴正方向为右
set(gca, 'YDir', 'reverse');  % 'reverse' 将 y 轴正向反转
