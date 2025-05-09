% function A = update_adjacency(x, r_comm)
%     N = size(x,1);
%     A = zeros(N,N);
%     for i = 1:N
%         for j = i+1:N
%             dist = norm(x(i,:) - x(j,:));
%             if dist <= r_comm
%                 A(i,j) = 1;
%                 A(j,i) = 1;  % 双向通信
%             end
%         end
%     end
% end

function A = update_adjacency(x, r_comm)
    N = 8; % 固定8个智能体
    A = zeros(N, N);
    
    % 1-2-3-4-5 五边形环状连通
    pentagon_nodes = 1:5;
    for k = 1:length(pentagon_nodes)
        i = pentagon_nodes(k);
        j = pentagon_nodes(mod(k, length(pentagon_nodes)) + 1); % 环形下一个节点
        A(i, j) = 1;
        A(j, i) = 1;
    end
    
    % 6-7-8 三角形环状连通
    triangle_nodes = 6:8;
    for k = 1:length(triangle_nodes)
        i = triangle_nodes(k);
        j = triangle_nodes(mod(k, length(triangle_nodes)) + 1); % 环形下一个节点
        A(i, j) = 1;
        A(j, i) = 1;
    end
    
    % 强制连接5与6（若间距 <= r_comm）
    dist_5_6 = norm(x(5, :) - x(6, :));
    if dist_5_6 <= r_comm
        A(5, 6) = 1;
        A(6, 5) = 1;
    end
    
    % 可选：对所有连接应用距离约束（若实际间距 > r_comm 则断开）
    for i = 1:N
        for j = i+1:N
            if A(i, j) == 1 % 仅检查已连接的边
                dist = norm(x(i, :) - x(j, :));
                if dist > r_comm
                    A(i, j) = 0;
                    A(j, i) = 0;
                end
            end
        end
    end
end