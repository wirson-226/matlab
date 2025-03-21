function new_pos = boundary_handling(pos, lb, ub, best_pos)
    % 新颖的边界处理方法
    
    new_pos = pos;
    [numRows, numCols] = size(pos);
   for j  = 1:numCols
        if new_pos(j) < lb(j)
            % 新位置基于最佳位置和一个随机扰动
            new_pos(j) = best_pos(j) + rand() * (lb(j) - best_pos(j));
        elseif new_pos(j) > ub(j)
            new_pos(j) = best_pos(j) + rand() * (ub(j) - best_pos(j));
        end
    end
end
