% modules/update_moving_obstacles.m
function pos = update_moving_obstacles(moving_obs)
    pos = moving_obs.pos + moving_obs.vel * 0.05;
end
