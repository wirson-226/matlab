function plot_group_error(state_hist, dt, desired_dist, ctrl)
    % 输入:
    %   state_hist: [T × N × D] 状态历史 (时间步×智能体×状态)
    %   dt: 时间步长
    %   desired_dist: 编队中理想距离
    %   ctrl: 控制参数结构体 (可选)
    
    [T, N, ~] = size(state_hist);
    t = (0:T-1) * dt;
    error_hist = zeros(T, 1);
    
    % ===== 1. 误差计算 =====
    for k = 1:T
        err = 0;
        count = 0;
        for i = 2:N
            for j = i+1:N
                xi = squeeze(state_hist(k,i,[1,3])); % 提取x,y坐标
                xj = squeeze(state_hist(k,j,[1,3]));
                dist = norm(xi - xj);
                err = err + (dist - desired_dist)^2;
                count = count + 1;
            end
        end
        error_hist(k) = sqrt(err / count); % RMSE
    end
    
    % ===== 2. 绘图设置 =====
    fig = figure('Name','Formation Error Analysis',...
                'Units','centimeters','Position',[6,6,18,12]);
    ax = axes(fig);
    
    % 主误差曲线
    main_plot = plot(ax, t, error_hist, 'b-',...
                    'LineWidth', 2,...
                    'DisplayName', 'Formation Error');
    hold(ax, 'on');
    
   
    % 收敛阈值线 (5%误差带)
    thresh_line = yline(ax, 0.05*desired_dist, ':g',...
                       'LineWidth', 1.2,...
                       'DisplayName', '5% Threshold');
    
    % ===== 3. 标注关键点 =====
    % 标记最终误差
    final_point = scatter(ax, t(end), error_hist(end), 100,...
                         'filled', 'MarkerFaceColor', 'r',...
                         'DisplayName', 'Final Error');
    
    % 标记最大误差
    [max_err, max_idx] = max(error_hist);
    max_point = scatter(ax, t(max_idx), max_err, 100,...
                       '^', 'MarkerFaceColor', 'm',...
                       'DisplayName', 'Max Error');
    
    % ===== 4. 图形美化 =====
    title(ax, sprintf('Formation Error (Desired Dist = %.2fm)', desired_dist));
    xlabel(ax, 'Time [s]', 'FontSize', 10);
    ylabel(ax, 'RMSE [m]', 'FontSize', 10);
    grid(ax, 'on');
    
    % 智能图例 (自动排除重复项)
    legend_items = [main_plot, thresh_line, final_point, max_point];
    legend(ax, legend_items,...
          'Location', 'northeastoutside',...
          'FontSize', 16,'FontName', 'Times New Roman');
    
    % ===== 5. 添加统计信息 =====
    stats_text = {
        sprintf('Avg Error: %.3f m', mean(error_hist))
        sprintf('Max Error: %.3f m', max_err)
        sprintf('Final Error: %.3f m', error_hist(end))
    };
    
    text(ax, 0.02, 0.95, stats_text,...
         'Units', 'normalized',...
         'FontSize', 16,...
         'FontName', 'Times New Roman',...
         'BackgroundColor', 'w',...
         'EdgeColor', [0.7 0.7 0.7],...  % 添加浅灰色边框
         'Margin', 2,...                  % 文本边距
         'VerticalAlignment', 'top',...
         'HorizontalAlignment', 'left');
    
    % ===== 6. 可选保存 =====
    if nargin > 3 && isfield(ctrl, 'save_plot') && ctrl.save_plot
        saveas(fig, 'formation_error_analysis.png');
    end
end