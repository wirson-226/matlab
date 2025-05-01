% modules/init_state.m
function [x, v, d_hat, d_hist, dhat_hist, state_hist] = init_state(num_agents, n_dim, steps)
    % x = 10 * rand(num_agents, n_dim);
    x = zeros(num_agents, n_dim);
    v = zeros(num_agents, n_dim);
    d_hat = zeros(num_agents, n_dim);
    d_hist = zeros(num_agents, n_dim, steps);
    dhat_hist = d_hist;
    state_hist.pos = zeros(num_agents, n_dim, steps);
    state_hist.vel = state_hist.pos;
end
