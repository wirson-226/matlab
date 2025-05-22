function plot_group_error(state_hist, dt, desired_dist)
    % 输入:
    %   state_hist: [T × N × D] 状态历史
    %   dt: 时间步长
    %   desired_dist: 编队中理想距离

    [T, N, D] = size(state_hist);
    t = (0:T-1) * dt;
    error_hist = zeros(1, T);

    for k = 1:T
        err = 0;
        count = 0;
        for i = 1:N
            for j = i+1:N
                xi = squeeze(state_hist(k,i,[1,3]));  % 取出 x, y
                xj = squeeze(state_hist(k,j,[1,3]));
                dist = norm(xi - xj);
                err = err + (dist - desired_dist)^2;
                count = count + 1;
            end
        end
        error_hist(k) = sqrt(err / count);  % RMSE
    end

    figure('Name','Formation Shape Error','Units','centimeters','Position',[6,6,16,9]);
    plot(t, error_hist, 'b-', 'LineWidth', 2);
    xlabel('Time [s]');
    ylabel('Shape Error (RMSE)');
    title('Formation Geometry Error Over Time');
    grid on;
end
