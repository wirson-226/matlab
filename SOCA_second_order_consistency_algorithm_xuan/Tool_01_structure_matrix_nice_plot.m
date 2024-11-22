% 定义L1和L2矩阵
L1 = [1 -1 0 0 0;
      0 1 -1 0 0;
      0 0 1 -1 0;
      0 0 0 1 -1;
     -1 0 0 0 1];
 
L2 = [2 -1 0 -1 0;
      0 2 -1 0 -1;
     -1 0 2 -1 0;
      0 -1 0 2 -1;
     -1 0 -1 0 2];

% 从拉普拉斯矩阵推导邻接矩阵
A1 = diag(diag(L1)) - L1;
A2 = diag(diag(L2)) - L2;

% 创建有向图
G1 = digraph(A1);
G2 = digraph(A2);

% 设置图形属性
node_size = 10; % 节点大小
arrow_size = 15; % 箭头大小
node_font_size = 12; % 节点标签字体大小

% 绘制L1对应的图
figure;
h1 = plot(G1, 'Layout', 'force', 'NodeLabel', {}, 'MarkerSize', node_size, 'LineWidth', 1.5, 'ArrowSize', arrow_size);
title('Directed Graph Topology from L1 Laplacian Matrix');
highlight(h1, 'Edges', find(G1.Edges.Weight > 0), 'LineStyle', '-', 'ArrowSize', arrow_size);

% 添加节点标签，调整标签位置
for i = 1:numnodes(G1)
    text(h1.XData(i), h1.YData(i), num2str(i), 'FontSize', node_font_size, 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom');
end

% 调整节点位置
h1.XData = h1.XData + rand(size(h1.XData)) * 0.1;
h1.YData = h1.YData + rand(size(h1.YData)) * 0.1;

% 绘制L2对应的图
figure;
h2 = plot(G2, 'Layout', 'force', 'NodeLabel', {}, 'MarkerSize', node_size, 'LineWidth', 1.5, 'ArrowSize', arrow_size);
title('Directed Graph Topology from L2 Laplacian Matrix');
highlight(h2, 'Edges', find(G2.Edges.Weight > 0), 'LineStyle', '-', 'ArrowSize', arrow_size);

% 添加节点标签，调整标签位置
for i = 1:numnodes(G2)
    text(h2.XData(i), h2.YData(i), num2str(i), 'FontSize', node_font_size, 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom');
end

% 调整节点位置
h2.XData = h2.XData + rand(size(h2.XData)) * 0.1;
h2.YData = h2.YData + rand(size(h2.YData)) * 0.1;
