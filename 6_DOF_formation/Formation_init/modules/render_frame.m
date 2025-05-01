% modules/render_frame.m
function frame = render_frame(x, targets, obstacles, r, adjacency)
    clf; hold on; axis equal;
    scatter(x(:,1), x(:,2), 50, 'b', 'filled');
    scatter(obstacles(:,1), obstacles(:,2), 100, 'r', 'filled');
    viscircles(obstacles, r*ones(size(obstacles,1),1), 'LineStyle','--');
    plot(targets(:,1), targets(:,2), 'k*');
    for i = 1:size(x,1)
        j = mod(i,size(x,1))+1;
        plot([x(i,1), x(j,1)], [x(i,2), x(j,2)], 'b--');
        plot([targets(i,1), targets(j,1)], [targets(i,2), targets(j,2)], 'k:');
    end
    % 可视化通信边
    for i = 1:size(x,1)
        for j = 1:size(x,1)
            if adjacency(i,j)
                plot([x(i,1), x(j,1)], [x(i,2), x(j,2)], 'g:');
            end
        end
    end

    xlim([-15, 15]); ylim([-15, 15]); title('Formation Frame');
    frame = getframe(gcf);
end