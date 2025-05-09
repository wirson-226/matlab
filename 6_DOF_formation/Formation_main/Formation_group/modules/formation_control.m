% modules/formation_control.m
function u_formation = formation_control(x, v, targets, ctrl)
    u_formation = -ctrl.kp_formation * (x - targets) - ctrl.kv_formation * v;
end


