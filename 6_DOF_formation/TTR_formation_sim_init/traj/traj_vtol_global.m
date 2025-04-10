function [ des_state ] = traj_vtol_global(t, state)
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
    if t <= t1 
        % 悬停阶段
        des_state.pos = [0; 0; 5];
        des_state.Va = 0;
        des_state.yaw = 0;
        des_state.mode = 1;
    elseif t <= t2
        % 悬停到巡航过渡 
        progress = (t - 3) / 3;
        des_state.pos = [progress * 20; 0; 5 + progress * 10];
        des_state.Va = 10 * progress;
        des_state.yaw = 0;
        des_state.mode = 2;
        % V2max = 10 * (t2 - 3) / 3;
        % 
        % if Va >= V2max
        %     des_state.mode = 3;
        % end

    elseif t <= t3
        % 巡航阶段
        des_state.pos = [20 + (t - 6); (t - 6) * 1; 15];
        des_state.Va = 10;
        des_state.yaw = atan2(des_state.pos(2) - 0, des_state.pos(1) - 20);
        des_state.mode = 3;
    
    elseif t <= t4
        % 巡航到悬停过渡
        progress = (t - 10) / 3;
        des_state.pos = [40 - progress * 20; progress * 10; 15 - progress * 10];
        des_state.Va = 10 * (1 - progress);
        des_state.yaw = 0;
        des_state.mode = 3;
        % if Va >= des_state.Va
        %     des_state.mode = 1;
        % end        
    
    else
        % 最终悬停
        des_state.pos = [20; 10; 5];
        des_state.Va = 0;
        des_state.yaw = 0;
        des_state.mode = 1;
    end
end