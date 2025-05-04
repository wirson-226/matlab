% modules/plot_disturbance_estimation.m（升级版）
function plot_disturbance_estimation(d_hist, dhat_hist, dt, ~)
    [N,~,T] = size(d_hist); t = (0:T-1)*dt;
    figure('Name','Disturbance Estimation','Units','centimeters','Position',[5,5,18,12]);
    for i = 1:N
        subplot(2,1,1); hold on;
        plot(t, squeeze(d_hist(i,1,:)), '-');
        plot(t, squeeze(dhat_hist(i,1,:)), '--');
        subplot(2,1,2); hold on;
        plot(t, squeeze(d_hist(i,2,:)), '-');
        plot(t, squeeze(dhat_hist(i,2,:)), '--');
    end
    subplot(2,1,1); title('X Disturbance Estimation'); grid on;
    subplot(2,1,2); title('Y Disturbance Estimation'); xlabel('Time [s]'); grid on;
end
