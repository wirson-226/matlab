function path = dubins_curve_3D(p1, p2, r, stepsize, h, quiet)
    % 计算二维 Dubins 路径
    path_2D = dubins_curve(p1, p2, r, stepsize, quiet);
    
    % 将路径从二维扩展到三维，z 坐标固定为定高 h
    path_3D = [path_2D, repmat(h, size(path_2D, 1), 1)];
    
    % 返回三维路径
    path = path_3D;
end
