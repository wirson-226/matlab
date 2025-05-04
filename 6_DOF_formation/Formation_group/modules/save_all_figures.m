% modules/save_all_figures.m（升级版）
function save_all_figures(output_dir)
    if nargin < 1, output_dir = '.'; end
    figs = findall(groot, 'Type', 'figure');
    for i = 1:length(figs)
        figure(figs(i));
        name = get(figs(i), 'Name');
        if isempty(name), name = sprintf('figure_%d', i); end
        
        % === 统一图像尺寸字体格式美化 ===
        format_figure_all_axes();
        
        % === 保存多格式图像 ===
        saveas(gcf, fullfile(output_dir, [name, '.eps']), 'epsc');
        % print(gcf, fullfile(output_dir, name), '-dpdf');  % 可启用 PDF
        print(gcf, fullfile(output_dir, name), '-dpng', '-r900');  % 高清
    end
end
