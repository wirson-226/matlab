clc
clear all
close all

%% 参数初始化定义
algorithms = {'DBO','QHDBO','GODBO'};
nPop = 30;                  %种群数
Max_iter = 10000;             %最大迭代次数
dim = 100;                   %可选 10、30、50、100
addpath("DBO变体")

for j = 1:2
    %% 选择函数
    Function_name = j; % 函数名： 1 - 30
    [lb, ub, dim, fobj] = Get_Functions_cec2017(Function_name, dim);
   
    %% DBO算法（灰狼算法）
    tic
    [DBO_Best_score, DBO_Best_pos, DBO_cg_curve] = DBO(nPop, Max_iter, lb, ub, dim, fobj);
    toc
    display(['The best optimal value of the objective function found by DBO for F' num2str(Function_name), ' is: ', num2str(DBO_Best_score)]);
    fprintf('Best solution obtained by DBO: %s\n', num2str(DBO_Best_pos, '%e  '));
    
   
    %% GODBO算法（灰狼算法）
    tic
    [QHDBO_Best_score, QHDBO_Best_pos, QHDBO_cg_curve] = QHDBO(nPop, Max_iter, lb, ub, dim, fobj);
    toc
    display(['The best optimal value of the objective function found by QHDBO for F' num2str(Function_name), ' is: ', num2str(QHDBO_Best_score)]);
    fprintf('Best solution obtained by QHDBO: %s\n', num2str(QHDBO_Best_pos, '%e  '));
    
    %% GODBO算法（灰狼算法）
    tic
    [GODBO_Best_score, GODBO_Best_pos, GODBO_cg_curve] = GODBO(nPop, Max_iter, lb, ub, dim, fobj);
    toc
    display(['The best optimal value of the objective function found by GODBO for F' num2str(Function_name), ' is: ', num2str(GODBO_Best_score)]);
    fprintf('Best solution obtained by GODBO: %s\n', num2str(GODBO_Best_pos, '%e  '));
    
    %% plot绘图
    figure;
    colors = lines(length(algorithms));
    for i = 1:length(algorithms)
        semilogy(log10(eval([algorithms{i} '_cg_curve'])), 'Color', colors(i, :), 'LineWidth', 1);
        hold on
    end
    title(['Convergence curve, dim=' num2str(dim)])
    xlabel('Iteration')
    ylabel(['Best score F' num2str(Function_name)])
    axis tight
    grid on
    box on
    set(gca, 'color', 'none')
    % Add legend
    legend(algorithms, 'Location', 'Best');
end