function set_figure_style(fig_handle, varargin)
% SET_FIGURE_STYLE 统一设置MATLAB图形输出格式
%   输入参数：
%       fig_handle - 图形句柄
%       varargin   - 可选参数对：
%           'FontName'    - 字体名称（默认：'Times New Roman'）
%           'FontSize'    - 基础字号（默认：12）
%           'LineWidth'   - 线宽（默认：1.2）
%           'DPI'         - 输出分辨率（默认：300）
%           'FigureSize'  - 图形尺寸[宽,高]（英寸）（默认：[6,4]）
%           'Color'       - 背景色（默认：'w'白色）
%           'Grid'        - 是否显示网格（默认：true）
%           'Box'         - 是否显示边框（默认：true）

    % 默认参数设置
    p = inputParser;
    addParameter(p, 'FontName', 'Times New Roman', @ischar);
    addParameter(p, 'FontSize', 12, @isnumeric);
    addParameter(p, 'LineWidth', 1.2, @isnumeric);
    addParameter(p, 'DPI', 300, @isnumeric);
    addParameter(p, 'FigureSize', [6,4], @(x)numel(x)==2);
    addParameter(p, 'Color', 'w', @(x)ischar(x)||isvector(x));
    addParameter(p, 'Grid', true, @islogical);
    addParameter(p, 'Box', true, @islogical);
    parse(p, varargin{:});
    
    % 设置图形属性
    set(fig_handle, ...
        'Color', p.Results.Color, ...
        'Units', 'inches', ...
        'Position', [0 0 p.Results.FigureSize], ...
        'PaperPositionMode', 'auto');
    
    % 获取所有坐标轴
    ax = findall(fig_handle, 'Type', 'axes');
    
    % 设置坐标轴属性
    for i = 1:length(ax)
        set(ax(i), ...
            'FontName', p.Results.FontName, ...
            'FontSize', p.Results.FontSize, ...
            'LineWidth', p.Results.LineWidth, ...
            'GridAlpha', 0.3, ...
            'XColor', [0.2 0.2 0.2], ...
            'YColor', [0.2 0.2 0.2], ...
            'ZColor', [0.2 0.2 0.2]);
        
        if p.Results.Grid, grid(ax(i), 'on'); else, grid(ax(i), 'off'); end
        if p.Results.Box, box(ax(i), 'on'); else, box(ax(i), 'off'); end
    end
    
    % 设置所有线条属性
    lines = findall(fig_handle, 'Type', 'line');
    for i = 1:length(lines)
        if lines(i).LineWidth < 1  % 保留小于1的线宽（如虚线）
            set(lines(i), 'LineWidth', max(0.5, lines(i).LineWidth));
        else
            set(lines(i), 'LineWidth', p.Results.LineWidth);
        end
    end
    
    % 设置文本属性
    texts = findall(fig_handle, 'Type', 'text');
    for i = 1:length(texts)
        set(texts(i), ...
            'FontName', p.Results.FontName, ...
            'FontSize', p.Results.FontSize);
    end
    
    % 设置图例属性
    legends = findall(fig_handle, 'Type', 'legend');
    for i = 1:length(legends)
        set(legends(i), ...
            'FontName', p.Results.FontName, ...
            'FontSize', p.Results.FontSize-1, ...
            'EdgeColor', [0.8 0.8 0.8]);
    end
    
    % 设置颜色条属性
    colorbars = findall(fig_handle, 'Type', 'colorbar');
    for i = 1:length(colorbars)
        set(colorbars(i), ...
            'FontName', p.Results.FontName, ...
            'FontSize', p.Results.FontSize-1);
    end
end