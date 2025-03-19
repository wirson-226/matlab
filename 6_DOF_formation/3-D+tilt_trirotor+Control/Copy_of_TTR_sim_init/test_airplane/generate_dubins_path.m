function full_path = generate_dubins_path(points, r, stepsize, h, quiet)
    % 生成多个点之间的 Dubins 路径
    % points: 一个 Nx3 矩阵，每行是一个点，包含 [x, y, theta]
    % r: 最小转弯半径
    % stepsize: 步长，决定路径的精细程度
    % h: 高度（默认为 0，表示平面内路径）
    % quiet: 是否安静模式，0 表示显示输出
    
    % 用于存储完整路径
    full_path = [];
    
    % 遍历每对相邻点，计算 Dubins 曲线并连接路径
    for i = 1:(size(points, 1) - 1)
        p1 = points(i, :);
        p2 = points(i + 1, :);
        
        % 调用 Dubins 曲线函数，计算相邻点之间的路径段
        path_segment = dubins_curve(p1, p2, r, stepsize,quiet);
        
        % 连接路径
        if isempty(full_path)
            full_path = path_segment;
        else
            full_path = [full_path; path_segment];
        end
    end
end
