function c = reference_trajectory(t)
    % 可定义为任意你想让编队中心跟踪的路径
    c = [5 * cos(0.5*t), 5 * sin(0.5*t)];  % 圆形轨迹
end