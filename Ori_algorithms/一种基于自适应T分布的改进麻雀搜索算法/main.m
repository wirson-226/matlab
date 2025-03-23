clc
clear all
close all

%% 参数初始化定义
algorithms = {'NTSSA','SSA'};
nPop = 30;                  %种群数
Max_iter = 500;             %最大迭代次数
dim = 50;                   %可选 10、30、50、100

for j = 1:30
    %% 选择函数
    Function_name = ['F' num2str(j)]; % 函数名： 1 - 30
    [lb, ub, dim, fobj] = Get_CEC2005_details(Function_name);
    %% NTSSA算法（灰狼算法）
    tic
    [NTSSA_Best_score, NTSSA_Best_pos, NTSSA_cg_curve] = NTSSA(nPop, Max_iter, lb, ub, dim, fobj);
    toc
    display(['The best optimal value of the objective function found by NTSSA for F' num2str(Function_name), ' is: ', num2str(NTSSA_Best_score)]);
    fprintf('Best solution obtained by NTSSA: %s\n', num2str(NTSSA_Best_pos, '%e  '));
    
     tic
    [SSA_Best_score, SSA_Best_pos, SSA_cg_curve] = SSA(nPop, Max_iter, lb, ub, dim, fobj);
    toc
    display(['The best optimal value of the objective function found by SSA for F' num2str(Function_name), ' is: ', num2str(SSA_Best_score)]);
    fprintf('Best solution obtained by SSA: %s\n', num2str(SSA_Best_pos, '%e  '));
   
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
