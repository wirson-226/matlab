% modules/plot_velocity_position_time.m
function plot_velocity_position_time(state_hist, dt, ~)
    [N, ~, T] = size(state_hist.pos); t = (0:T-1) * dt;
    figure('Name','Position','Units','centimeters','Position',[5,5,18,10]);
    for i = 1:N
        subplot(3,1,1); hold on; plot(t, squeeze(state_hist.pos(i,1,:)));
        subplot(3,1,2); hold on; plot(t, squeeze(state_hist.pos(i,2,:)));
        subplot(3,1,3); hold on; plot(t, vecnorm(squeeze(state_hist.pos(i,:,:))', 2, 2));
    end
    subplot(3,1,1); title('X Position'); grid on;
    subplot(3,1,2); title('Y Position'); grid on;
    subplot(3,1,3); title('Position Magnitude'); grid on; xlabel('Time [s]');
    figure('Name','Velocity','Units','centimeters','Position',[5,5,18,10]);
    for i = 1:N
        subplot(3,1,1); hold on; plot(t, squeeze(state_hist.vel(i,1,:)));
        subplot(3,1,2); hold on; plot(t, squeeze(state_hist.vel(i,2,:)));
        subplot(3,1,3); hold on; plot(t, vecnorm(squeeze(state_hist.vel(i,:,:))', 2, 2));
    end
    subplot(3,1,1); title('X Velocity'); grid on;
    subplot(3,1,2); title('Y Velocity'); grid on;
    subplot(3,1,3); title('Velocity Magnitude'); grid on; xlabel('Time [s]');
end