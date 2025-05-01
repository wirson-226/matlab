% modules/save_all_figures.m（升级版）
function save_all_figures(output_dir)
    if nargin < 1, output_dir = '.'; end
    figs = findall(groot, 'Type', 'figure');
    for i = 1:length(figs)
        figure(figs(i));
        name = get(figs(i), 'Name');
        if isempty(name), name = sprintf('figure_%d', i); end
        saveas(figs(i), fullfile(output_dir, [name, '.eps']), 'epsc');
        print(gcf, fullfile(output_dir, name), '-dpdf');
        print(gcf, fullfile(output_dir, name), '-dpng', '-r300');
    end
end