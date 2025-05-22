%% 修改后的状态初始化
function [x, v, d_hat, d_hist, dhat_hist, state_hist] = init_state(num_agents, n_dim, steps)
    % 状态向量：[x, vx, y, vy] for each agent
    x = zeros(num_agents, n_dim);
    v = zeros(num_agents, n_dim); % 这里可能不再需要，因为速度已包含在x中
    
    % 初始化leader位置（智能体1）
    x(1, :) = [0, 0, 0, 0];  % [x, vx, y, vy]
    
    % 初始化followers位置（智能体2,3,4）
    x(2, :) = [0.5, 0, -0.5, 0];
    x(3, :) = [-0.5, 0, -0.5, 0];
    x(4, :) = [0, 0, 0.5, 0];
    
    % 其他初始化
    d_hat = zeros(num_agents, n_dim);
    d_hist = zeros(steps, num_agents, n_dim);
    dhat_hist = zeros(steps, num_agents, n_dim);
    state_hist = zeros(steps, num_agents, n_dim);
end