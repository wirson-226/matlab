%% 修改后的参数初始化
function [num_agents, n_dim, dt, T_total, u_max, r_safe, formation_mode, ctrl, gamma, eta, d0, comm_radius] = init_params()
    % 核心修改：固定为4个智能体（1 leader + 3 followers）
    num_agents = 4;
    n_dim = 4;       % 每个智能体状态维度：[x, vx, y, vy]
    dt = 0.01;
    T_total = 60;
    u_max = 2.0;
    r_safe = 0.7;
    formation_mode = 'triangle';
    
    % 原编队协议的特定参数
    ctrl.w = 0.314; %角速度
    ctrl.r = 1.5;   %半径
    ctrl.w_lead = 0.157;
    ctrl.r_lead = 0.5;
    ctrl.K1 = [-2, -1.2];
    ctrl.K_1 = [-1, 0];
    ctrl.K_2 = [-0.1251, -0.5732]*2;
    ctrl.c_1 = 0.5*2;
    ctrl.c_2 = 1*2;
    
    % APF参数
    ctrl.k_rep = 1.1;
    ctrl.k_rot = 1.2;
    ctrl.k_damp = 0.5;
    ctrl.r_safe = r_safe;
    
    gamma = 1.0;
    eta = 0.5;
    d0 = 0.3;
    comm_radius = 2.0;
end