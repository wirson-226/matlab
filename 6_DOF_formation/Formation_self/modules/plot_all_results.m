% modules/plot_all_results.m
function plot_all_results(state_hist, d_hist, dhat_hist, targets, dt, output_dir)
    if nargin < 6, output_dir = './results'; end
    if ~exist(output_dir, 'dir'), mkdir(output_dir); end

    plot_trajectory(state_hist.pos, output_dir);
    plot_velocity_position_time(state_hist, dt, output_dir);
    plot_disturbance_estimation(d_hist, dhat_hist, dt, output_dir);
    % plot_disturbance_error_summary(d_hist, dhat_hist, dt, output_dir);
    % if ~isempty(targets)
    %     plot_group_error(state_hist.pos, targets, dt, output_dir);
    % end
    save_all_figures(output_dir);
end