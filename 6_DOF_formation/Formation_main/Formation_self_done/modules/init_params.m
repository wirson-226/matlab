% modules/init_params.m
function [num_agents, n_dim, dt, T_total, u_max, r_safe, formation_mode, ctrl, gamma, eta, d0, comm_radius] = init_params()
    num_agents = 6;
    n_dim = 2;
    dt = 0.05;
    T_total = 2;
    u_max = 2.0;  % input limit
    eta = 0.005;    % APF 避障增益
    d0 = 5;     % APF 激活距离
    r_safe = 10;     % CBF 安全距离
    gamma = 1;    % CBF 避障优化系数
    comm_radius = 10;  % 通信半径（可调）

    formation_mode = 'rotating';
    ctrl.kp_formation = 10;
    ctrl.kv_formation = 3;
    ctrl.beta_consensus = 0.5;
    ctrl.gamma_observer = 0.1;
end