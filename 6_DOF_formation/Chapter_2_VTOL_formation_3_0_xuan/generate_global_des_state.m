function global_des_state = generate_global_des_state(trajhandle, t, num_agents)
% [13 * num_agents] 
global_des_state.pos = zeros(3, num_agents);
global_des_state.vel = zeros(3, num_agents);
global_des_state.acc = zeros(3, num_agents);
global_des_state.yaw = zeros(1, num_agents);
global_des_state.yawdot = zeros(1, num_agents);

for agent = 1:num_agents
    des_state = trajhandle(t, agent);
    global_des_state.pos(:, agent) = des_state.pos;
    global_des_state.vel(:, agent) = des_state.vel;
    global_des_state.acc(:, agent) = des_state.acc;
    global_des_state.yaw(agent) = des_state.yaw;
    global_des_state.yawdot(agent) = des_state.yawdot;
end
end