%% 通信拓扑更新函数
function adjacency = update_adjacency(x, comm_radius)
    % 基于距离的通信拓扑
    num_agents = size(x, 1);
    adjacency = zeros(num_agents, num_agents);
    
    for i = 1:num_agents
        for j = 1:num_agents
            if i ~= j
                pos_i = [x(i, 1), x(i, 3)];  % [x, y]
                pos_j = [x(j, 1), x(j, 3)];
                dist = norm(pos_i - pos_j);
                
                if dist <= comm_radius
                    adjacency(i, j) = 1;
                end
            end
        end
    end
end