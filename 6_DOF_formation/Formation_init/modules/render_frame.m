% % modules/render_frame.m -- 持久窗口版本
% function frame = render_frame(x, targets, obstacles, r, adjacency, is_final, group_ids, centers)
%     persistent fig_handle;
%     if isempty(fig_handle) || ~isvalid(fig_handle)
%         fig_handle = figure('Name', 'Formation Visualization', 'NumberTitle', 'off', ...
%                             'Units', 'centimeters', 'Position', [3, 3, 22, 18]);
%     else
%         figure(fig_handle);
%     end
% 
%     clf(fig_handle); hold on; axis equal;
%     [N, dim] = size(x);
%     colororder = lines(N);
% 
%     h_agent = gobjects(N,1);
%     h_target = gobjects(N,1);
%     for i = 1:N
%         color = colororder(i,:);
%         h_agent(i) = scatter(x(i,1), x(i,2), 100, 'o', 'filled', 'MarkerFaceColor', color, 'MarkerEdgeColor', color);
%         h_target(i) = scatter(targets(i,1), targets(i,2), 100, 'o', 'MarkerFaceColor', 'none', 'MarkerEdgeColor', color, 'LineWidth', 1.5);
%     end
% 
%     scatter(obstacles(:,1), obstacles(:,2), 50, 'r', 'filled');
%     viscircles(obstacles, r*ones(size(obstacles,1),1), 'LineStyle','--','Color','r');
% 
%     unique_groups = unique(group_ids);
%     for g = 1:length(unique_groups)
%         members = find(group_ids == unique_groups(g));
%         for i = 1:length(members)
%             for j = i+1:length(members)
%                 ii = members(i); jj = members(j);
%                 if adjacency(ii, jj)
%                     plot([x(ii,1), x(jj,1)], [x(ii,2), x(jj,2)], 'g-','LineWidth', 1);
%                 end
%             end
%         end
%     end
% 
%     for g = 1:length(unique_groups)
%         members = find(group_ids == unique_groups(g));
%         for i = 1:length(members)
%             j = members(mod(i, length(members)) + 1);
%             plot([x(members(i),1), x(j,1)], [x(members(i),2), x(j,2)], 'b--','LineWidth', 1.2);
%             plot([targets(members(i),1), targets(j,1)], [targets(members(i),2), targets(j,2)], 'k--','LineWidth', 1);
%         end
%     end
% 
%     h_group_centers = gobjects(length(unique_groups), 1);
%     if nargin >= 8 && ~isempty(group_ids)
%         group_colors = lines(length(unique_groups));
%         for g = 1:length(unique_groups)
%             center = centers(g,:);
%             h_group_centers(g) = scatter(center(1), center(2), 150, 'p', 'MarkerEdgeColor', group_colors(g,:), 'MarkerFaceColor', 'y', 'LineWidth', 1.5);
%         end
%     end
% 
%     xlim([-30, 30]); ylim([-30, 30]);
%     xlabel('X [m]'); ylabel('Y [m]');
%     if is_final
%         title(fig_handle.CurrentAxes, 'Final Formation Frame');
%     else
%         title(fig_handle.CurrentAxes, 'Formation Frame');
%     end
% 
% 
%     if is_final
%         plot(nan, nan, 'b--','LineWidth', 1.2 ,'DisplayName', 'Actual Formation');
%         plot(nan, nan, 'k--','LineWidth', 1 , 'DisplayName', 'Target Formation');
%         plot(nan, nan, 'g-','LineWidth', 1 , 'DisplayName', 'Dynamic Connection');
%         plot(nan, nan, 'r--','LineWidth', 1.5 , 'DisplayName', 'Obstacle Boundary');
%         plot(nan, nan, 'ro',  'MarkerFaceColor', 'r', 'DisplayName', 'Obstacle Center');
%         plot(nan, nan, 'p',   'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k', 'DisplayName', 'Group Center');
% 
%         legend([h_agent(1), h_target(1), ...
%                 findobj(gca,'DisplayName','Obstacle Center'), ...
%                 findobj(gca,'DisplayName','Obstacle Boundary'), ...
%                 findobj(gca,'DisplayName','Group Center'), ...
%                 findobj(gca,'DisplayName','Actual Formation'), ...
%                 findobj(gca,'DisplayName','Target Formation'), ...
%                 findobj(gca,'DisplayName','Dynamic Connection')], ...
%                {'Agent', 'Target', 'Obstacle Center', 'Obstacle Boundary', 'Group Center', 'Actual Formation', 'Target Formation', 'Dynamic Connection'}, 'Location', 'bestoutside');
%     end
% 
%     frame = getframe(fig_handle);
% end


% modules/render_frame.m -- 持久窗口版本
function frame = render_frame(x, targets, obstacles, r, adjacency, is_final, group_ids, centers)
    persistent fig_handle;
    if isempty(fig_handle) || ~isvalid(fig_handle)
        fig_handle = figure('Name', 'Formation Visualization', 'NumberTitle', 'off', ...
                            'Units', 'centimeters', 'Position', [3, 3, 22, 18]);
    else
        figure(fig_handle);
    end

    clf(fig_handle); hold on; axis equal;
    [N, dim] = size(x);
    colororder = lines(N);

    h_agent = gobjects(N,1);
    h_target = gobjects(N,1);
    for i = 1:N
        color = colororder(i,:);
        h_agent(i) = scatter(x(i,1), x(i,2), 100, 'o', 'filled', 'MarkerFaceColor', color, 'MarkerEdgeColor', color);
        h_target(i) = scatter(targets(i,1), targets(i,2), 100, 'o', 'MarkerFaceColor', 'none', 'MarkerEdgeColor', color, 'LineWidth', 1.5);
    end

    scatter(obstacles(:,1), obstacles(:,2), 50, 'r', 'filled');
    viscircles(obstacles, r*ones(size(obstacles,1),1), 'LineStyle','--','Color','r');

    % ==== 所有动态连接 ==== 
    for i = 1:N
        for j = i+1:N
            if adjacency(i,j)
                plot([x(i,1), x(j,1)], [x(i,2), x(j,2)], 'g-', 'LineWidth', 1);
            end
        end
    end

    % ==== 分组内部编队连线 ====
    unique_groups = unique(group_ids);
    for g = 1:length(unique_groups)
        members = find(group_ids == unique_groups(g));
        for i = 1:length(members)
            j = members(mod(i, length(members)) + 1);
            plot([x(members(i),1), x(j,1)], [x(members(i),2), x(j,2)], 'b--','LineWidth', 1.2);
            plot([targets(members(i),1), targets(j,1)], [targets(members(i),2), targets(j,2)], 'k--','LineWidth', 1);
        end
    end

    h_group_centers = gobjects(length(unique_groups), 1);
    if nargin >= 8 && ~isempty(group_ids)
        group_colors = lines(length(unique_groups));
        for g = 1:length(unique_groups)
            center = centers(g,:);
            h_group_centers(g) = scatter(center(1), center(2), 150, 'p', 'MarkerEdgeColor', group_colors(g,:), 'MarkerFaceColor', 'y', 'LineWidth', 1.5);
        end
    end

    xlim([-30, 30]); ylim([-30, 30]);
    xlabel('X [m]'); ylabel('Y [m]');
    if is_final
        title(fig_handle.CurrentAxes, 'Final Formation Frame');
    else
        title(fig_handle.CurrentAxes, 'Formation Frame');
    end

    if is_final
        plot(nan, nan, 'b--','LineWidth', 1.2 ,'DisplayName', 'Actual Formation');
        plot(nan, nan, 'k--','LineWidth', 1 , 'DisplayName', 'Target Formation');
        plot(nan, nan, 'g-','LineWidth', 1 , 'DisplayName', 'Dynamic Connection');
        plot(nan, nan, 'r--','LineWidth', 1.5 , 'DisplayName', 'Obstacle Boundary');
        plot(nan, nan, 'ro',  'MarkerFaceColor', 'r', 'DisplayName', 'Obstacle Center');
        plot(nan, nan, 'p',   'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k', 'DisplayName', 'Group Center');

        legend([h_agent(1), h_target(1), ...
                findobj(gca,'DisplayName','Obstacle Center'), ...
                findobj(gca,'DisplayName','Obstacle Boundary'), ...
                findobj(gca,'DisplayName','Group Center'), ...
                findobj(gca,'DisplayName','Actual Formation'), ...
                findobj(gca,'DisplayName','Target Formation'), ...
                findobj(gca,'DisplayName','Dynamic Connection')], ...
               {'Agent', 'Target', 'Obstacle Center', 'Obstacle Boundary', 'Group Center', 'Actual Formation', 'Target Formation', 'Dynamic Connection'}, 'Location', 'bestoutside');
    end

    frame = getframe(fig_handle);
end

