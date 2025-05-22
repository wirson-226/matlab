function u_sat = saturate(u, low_limit, up_limit)
    % 当传入是向量时，经过饱和约束后不改变向量的方向；当传入是标量时，执行正常的饱和策略
    % Args:
    %     u: 输入量
    %     low_limit: 下界
    %     up_limit: 上界
    % Returns:
    %     经过饱和处理后的输入量u_sat
    
    if isscalar(u)
        % Scalar case: Apply normal saturation
        if u <= low_limit
            u_sat = low_limit;
        elseif u >= up_limit
            u_sat = up_limit;
        else
            u_sat = u;
        end
    else
        % Vector case: Preserve the direction but apply saturation on the norm
        norm_u = norm(u);
        if norm_u == 0
            u_sat = u; % If the vector is zero, return it unchanged
        elseif norm_u <= low_limit
            u_sat = low_limit * u / norm_u; % Scale down if norm is too small
        elseif norm_u >= up_limit
            u_sat = up_limit * u / norm_u; % Scale up if norm is too large
        else
            u_sat = u; % No saturation required
        end
    end
end
