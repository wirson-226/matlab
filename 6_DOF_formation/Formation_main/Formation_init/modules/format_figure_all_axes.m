function format_figure_all_axes()
    set(gcf, 'Units', 'centimeters', 'Position', [5, 5, 36, 20]);
    axes_handles = findall(gcf, 'Type', 'axes');
    for ax = axes_handles'
        set(ax, 'FontName', 'Times New Roman', 'FontSize', 16, 'LineWidth', 0.5);
        grid(ax, 'on');
        box(ax, 'on');
    end
    legend('FontSize', 16);
end
