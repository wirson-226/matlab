clc
clear all
close all

%% 参数初始化定义
algorithms = {'LPO','ATK'};
nPop = 30;                  %种群数
Max_iter = 500;             %最大迭代次数
dim = 50;                   %可选 10、30、50、100

for j = 1:30
    %% 选择函数
    % Function_name = ['F' num2str(j)]; % 函数名： 1 - 30
    Function_name  = j;
    [lb, ub, dim, fobj] = Get_CEC2017_details(Function_name,dim);
    %% LPO算法（灰狼算法）
    tic
    [LPO_Best_score, LPO_Best_pos, LPO_cg_curve] = LPO(nPop, Max_iter, lb, ub, dim, fobj);
    toc
    display(['The best optimal value of the objective function found by LPO for F' num2str(Function_name), ' is: ', num2str(LPO_Best_score)]);
    fprintf('Best solution obtained by LPO: %s\n', num2str(LPO_Best_pos, '%e  '));
    
       %% ATK算法（灰狼算法）
    tic
    [ATK_Best_score, ATK_Best_pos, ATK_cg_curve] = ATK(nPop, Max_iter, lb, ub, dim, fobj);
    toc
    display(['The best optimal value of the objective function found by ATK for F' num2str(Function_name), ' is: ', num2str(ATK_Best_score)]);
    fprintf('Best solution obtained by ATK: %s\n', num2str(ATK_Best_pos, '%e  '));
    
    %% plot绘图
    figure;
    colors = lines(length(algorithms));
    for i = 1:length(algorithms)
        semilogy(eval([algorithms{i} '_cg_curve']), 'Color', colors(i, :), 'LineWidth', 1);
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
