% modules/render_frame.m
function frame = render_frame(x, targets, obstacles, r, adjacency, is_final)
    clf; hold on; axis equal;
    [N, dim] = size(x);
    colororder = lines(N);

    h_agent = gobjects(N,1);
    h_target = gobjects(N,1);
    for i = 1:N
        color = colororder(i,:);
        h_agent(i) = scatter(x(i,1), x(i,2), 100, 'o', 'filled', 'MarkerFaceColor', color, 'MarkerEdgeColor', color);
        h_target(i) = scatter(targets(i,1), targets(i,2), 100, 'o', 'MarkerFaceColor', 'none', 'MarkerEdgeColor', color, 'LineWidth', 1.5);
    end

    scatter(obstacles(:,1), obstacles(:,2), 100, 'r', 'filled');
    viscircles(obstacles, r*ones(size(obstacles,1),1), 'LineStyle','--','Color','r');

    for i = 1:N
        for j = i+1:N
            if adjacency(i,j)
                plot([x(i,1), x(j,1)], [x(i,2), x(j,2)], 'g-','LineWidth', 1);
            end
        end
    end
    for i = 1:N
        j = mod(i, N) + 1;
        plot([x(i,1), x(j,1)], [x(i,2), x(j,2)], 'b--','LineWidth', 1.2);
        plot([targets(i,1), targets(j,1)], [targets(i,2), targets(j,2)], 'k--','LineWidth', 1);
    end

    % 绘制编队中心
    center = mean(targets, 1);
    h_center = scatter(center(1), center(2), 150, 'p', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'y', 'LineWidth', 1.5);  % 五角星黄色中心

    xlim([-15, 15]); ylim([-15, 15]);
    xlabel('X [m]'); ylabel('Y [m]');
    if is_final
        set(gcf, 'Name', 'Final Frame', 'NumberTitle', 'off');
        title('Final Formation Frame');
    else
        set(gcf, 'Name', 'Simulation Frame', 'NumberTitle', 'off');
        title('Formation Frame');
    end

    if nargin > 5 && is_final
        % 添加图例（仅在最后一帧）
        plot(nan, nan, 'b--','LineWidth', 1.2 ,'DisplayName', 'Actual Formation'); % 实时编队形状
        plot(nan, nan, 'k--','LineWidth', 1 , 'DisplayName', 'Target Formation'); % 目标编队形状
        plot(nan, nan, 'g-','LineWidth', 1 , 'DisplayName', 'Dynamic Connection'); % 动态通讯连接
        plot(nan, nan, 'r--','LineWidth', 1.5 , 'DisplayName', 'Obstacle Boundary'); % 障碍物边缘
        plot(nan, nan, 'ro',  'MarkerFaceColor', 'r', 'DisplayName', 'Obstacle Center'); % 障碍物中心
        plot(nan, nan, 'p',   'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k', 'DisplayName', 'Formation Center'); % 编队中心

        legend([h_agent(1), h_target(1), ...
                findobj(gca,'DisplayName','Obstacle Center'), ...
                findobj(gca,'DisplayName','Obstacle Boundary'), ...
                findobj(gca,'DisplayName','Formation Center'), ...
                findobj(gca,'DisplayName','Actual Formation'), ...
                findobj(gca,'DisplayName','Target Formation'), ...
                findobj(gca,'DisplayName','Dynamic Connection')], ...
               {'Agent', 'Target', 'Obstacle Center', 'Obstacle Boundary', 'Formation Center', 'Actual Formation', 'Target Formation', 'Dynamic Connection'}, 'Location', 'bestoutside');
    end

    frame = getframe(gcf);
%     %% 高分辨率视频 %%
%     % exportgraphics(gca, 'temp_frame.png', 'Resolution', 300);
%     % img = imread('temp_frame.png');
%     % frame = im2frame(img);
end

