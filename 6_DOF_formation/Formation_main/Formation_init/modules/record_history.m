% modules/record_history.m
function [sh, dh, dhat] = record_history(sh, dh, dhat, x, v, d, d_est, step)
    sh.pos(:,:,step) = x;
    sh.vel(:,:,step) = v;
    dh(:,:,step) = d;
    dhat(:,:,step) = d_est;
end