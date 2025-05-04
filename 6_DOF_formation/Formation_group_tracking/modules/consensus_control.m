% modules/consensus_control.m
function u_consensus = consensus_control(x, v, d_hat, adjacency, ctrl, u_max)
    num_agents = size(x,1);
    u_consensus = zeros(size(x));
    for i = 1:num_agents
        for j = 1:num_agents
            if adjacency(i,j)
                u_consensus(i,:) = u_consensus(i,:) - (x(i,:) - x(j,:)) - ctrl.beta_consensus*(v(i,:) - v(j,:));
            end
        end
        d_hat(i,:) = d_hat(i,:) - ctrl.gamma_observer * u_consensus(i,:);
    end
    u_consensus = u_consensus - d_hat;
    u_consensus = max(min(u_consensus, u_max), -u_max);
end