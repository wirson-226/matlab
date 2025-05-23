% modules/plot_all_results.m
function plot_all_results(state_hist, desired_dist, static_obs, dt, output_dir)
    if nargin < 6, output_dir = './results'; end
    if ~exist(output_dir, 'dir'), mkdir(output_dir); end

    plot_trajectory(state_hist,static_obs);
    plot_velocity_position_time(state_hist, dt, output_dir);
    plot_group_error(state_hist, dt, desired_dist);

    save_all_figures(output_dir);
end