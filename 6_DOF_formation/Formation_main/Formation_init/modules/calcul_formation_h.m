% formation shape
function h = calcul_formation_h(w,r,time)
    t = time;
    h = zeros(16, 1);  % 4 agents × 4 state = 16

    % 定义菱形相对坐标（顺序依次为：前 → 左 → 右 → 后）
    rel_pos = [
         -5,  5;   % Agent 1：Leader（在前）
         -5,  0;   % Agent 2：左下
         -5,  -5;   % Agent 3：右下
         -5,  -10    % Agent 4：尾部
    ];


    for i = 1:4
        h((4*(i-1)+1):4*i,:) = [rel_pos(i,1);  % x
                                0;                   % vx
                                rel_pos(i,2);        % y
                                0];                  % vy
    end
    % disp(h);
end