% modules/plot_velocity_position_time.m
function plot_velocity_position_time(state_hist, dt, ~)
    [N, ~, T] = size(state_hist.pos); t = (0:T-1) * dt;
    colororder = lines(N);

    % Position plot with tiledlayout
    figure('Name','Position','Units','centimeters','Position',[5,5,18,10]);
    tl1 = tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
    for i = 1:N
        nexttile(1); hold on;
        plot(t, squeeze(state_hist.pos(i,1,:)), 'Color', colororder(i,:), 'LineWidth', 0.8, 'DisplayName', sprintf('Agent %d', i));

        nexttile(2); hold on;
        plot(t, squeeze(state_hist.pos(i,2,:)), 'Color', colororder(i,:), 'LineWidth', 0.8,'DisplayName', sprintf('Agent %d', i));

        nexttile(3); hold on;
        plot(t, vecnorm(squeeze(state_hist.pos(i,:,:))', 2, 2), 'Color', colororder(i,:),'LineWidth', 0.8, 'DisplayName', sprintf('Agent %d', i));
    end
    nexttile(1); title('X Position'); grid on; ylabel('X Position [m]'); legend('show','Location','northeastoutside');
    nexttile(2); title('Y Position'); grid on; ylabel('Y Position [m]');legend('show','Location','northeastoutside');
    nexttile(3); title('Position Magnitude'); grid on; xlabel('Time [s]'); ylabel('Magnitude [m]');legend('show','Location','northeastoutside');

    % Velocity plot with tiledlayout
    figure('Name','Velocity','Units','centimeters','Position',[5,5,18,10]);
    tl2 = tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
    for i = 1:N
        nexttile(1); hold on;
        plot(t, squeeze(state_hist.vel(i,1,:)), 'Color', colororder(i,:),'LineWidth', 0.8, 'DisplayName', sprintf('Agent %d', i));

        nexttile(2); hold on;
        plot(t, squeeze(state_hist.vel(i,2,:)), 'Color', colororder(i,:), 'LineWidth', 0.8,'DisplayName', sprintf('Agent %d', i));

        nexttile(3); hold on;
        plot(t, vecnorm(squeeze(state_hist.vel(i,:,:))', 2, 2), 'Color', colororder(i,:),'LineWidth', 0.8,'DisplayName', sprintf('Agent %d', i));
    end
    nexttile(1); title('X Velocity'); grid on; ylabel('X Velocity [m/s]'); legend('show','Location','northeastoutside');
    nexttile(2); title('Y Velocity'); grid on; ylabel('Y Velocity [m/s]');legend('show','Location','northeastoutside');
    nexttile(3); title('Velocity Magnitude'); grid on; xlabel('Time [s]'); ylabel('Magnitude [m/s]');legend('show','Location','northeastoutside');
end