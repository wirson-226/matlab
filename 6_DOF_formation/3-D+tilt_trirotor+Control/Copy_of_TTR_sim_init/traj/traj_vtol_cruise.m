function [ des_state ] = traj_vtol_cruise(t, state)
    % 创建VTOL飞行器测试轨迹的期望状态
    % u_r = state.vel(1) - 0;
    % v_r = state.vel(2) - 0;
    % w_r = state.vel(3) - 0;
    % Va = sqrt(u_r^2 + v_r^2 + w_r^2);
    t1 = 3;
    t2 = 6;
    t3 = 10;
    t4 = 13;
    t5 = 15;

    % 阶段定义
    if t <= 100
        % 巡航阶段
        % des_state.pos = [2*t; 2*t; 10];
         des_state.Va = 15;
         % des_state.pos = [20; 20+des_state.Va * t; 10];
         des_state.pos = [200; 200; 10];
        
        % des_state.yaw = atan2(des_state.pos(2) - 0, des_state.pos(1) - 20);
        des_state.mode = 3;   
   
    
    else
        % 巡航阶段
        % des_state.pos = [2*t; 2*t; 10];
         des_state.Va = 15;
         des_state.pos = [20; 20+des_state.Va * t; 0];
        
        % des_state.yaw = atan2(des_state.pos(2) - 0, des_state.pos(1) - 20);
        des_state.mode = 3;  
    end
end