clc
clear all
close all

%% 参数初始化定义
algorithms = {'PSO','IDWPSO','IPSO','MeanPSO','MPSO','MSCPSO','P_PSO_SA','SLPSO'};
nPop = 30;                  %种群数
Max_iter = 500;             %最大迭代次数
dim = 100 ;                   %可选 10、30、50、100

for j = 1:9
    %% 选择函数
    Function_name =  ['F' num2str(j)]; % 函数名： 1 - 30
    [lb, ub, dim, fobj] = Get_CEC2005_details(Function_name);
        %% PSO算法
    tic
    [PSO_Best_score, PSO_Best_pos, PSO_cg_curve] = PSO(nPop, Max_iter, lb, ub, dim, fobj);
    toc
    display(['The best optimal value of the objective function found by PSO for F' num2str(Function_name), ' is: ', num2str(PSO_Best_score)]);
    fprintf('Best solution obtained by PSO: %s\n', num2str(PSO_Best_pos, '%e  '));

    
            %% IPSO算法
    tic
    [IPSO_Best_score, IPSO_Best_pos, IPSO_cg_curve] = IPSO(nPop, Max_iter, lb, ub, dim, fobj);
    toc
    display(['The best optimal value of the objective function found by IPSO for F' num2str(Function_name), ' is: ', num2str(IPSO_Best_score)]);
    fprintf('Best solution obtained by IPSO: %s\n', num2str(IPSO_Best_pos, '%e  '));

     %% IDWPSO算法
    tic
    [IDWPSO_Best_score, IDWPSO_Best_pos, IDWPSO_cg_curve] = IDWPSO(nPop, Max_iter, lb, ub, dim, fobj);
    toc
    display(['The best optimal value of the objective function found by IDWPSO for F' num2str(Function_name), ' is: ', num2str(IDWPSO_Best_score)]);
    fprintf('Best solution obtained by IDWPSO: %s\n', num2str(IDWPSO_Best_pos, '%e  '));
    
    %% MeanPSO算法
    tic
    [MeanPSO_Best_score, MeanPSO_Best_pos, MeanPSO_cg_curve] = MeanPSO(nPop, Max_iter, lb, ub, dim, fobj);
    toc
    display(['The best optimal value of the objective function found by MeanPSO for F' num2str(Function_name), ' is: ', num2str(MeanPSO_Best_score)]);
    fprintf('Best solution obtained by MeanPSO: %s\n', num2str(MeanPSO_Best_pos, '%e  '));

        %% MPSO算法
    tic
    [MPSO_Best_score, MPSO_Best_pos, MPSO_cg_curve] = MPSO(nPop, Max_iter, lb, ub, dim, fobj);
    toc
    display(['The best optimal value of the objective function found by MPSO for F' num2str(Function_name), ' is: ', num2str(MPSO_Best_score)]);
    fprintf('Best solution obtained by MPSO: %s\n', num2str(MPSO_Best_pos, '%e  '));

    %% MSCPSO算法
    tic
    [MSCPSO_Best_score, MSCPSO_Best_pos, MSCPSO_cg_curve] = MSCPSO(nPop, Max_iter, lb, ub, dim, fobj);
    toc
    display(['The best optimal value of the objective function found by MSCPSO for F' num2str(Function_name), ' is: ', num2str(MSCPSO_Best_score)]);
    fprintf('Best solution obtained by MSCPSO: %s\n', num2str(MSCPSO_Best_pos, '%e  '));

        %% P_PSO_SA算法
    tic
    [P_PSO_SA_Best_score, P_PSO_SA_Best_pos, P_PSO_SA_cg_curve] = P_PSO_SA(nPop, Max_iter, lb, ub, dim, fobj);
    toc
    display(['The best optimal value of the objective function found by P_PSO_SA for F' num2str(Function_name), ' is: ', num2str(P_PSO_SA_Best_score)]);
    fprintf('Best solution obtained by P_PSO_SA: %s\n', num2str(P_PSO_SA_Best_pos, '%e  '));

        %% SLPSO算法
    tic
    [SLPSO_Best_score, SLPSO_Best_pos, SLPSO_cg_curve] = SLPSO(nPop, Max_iter, lb, ub, dim, fobj);
    toc
    display(['The best optimal value of the objective function found by SLPSO for F' num2str(Function_name), ' is: ', num2str(SLPSO_Best_score)]);
    fprintf('Best solution obtained by SLPSO: %s\n', num2str(SLPSO_Best_pos, '%e  '));


    %% plot绘图
    figure
    colors = lines(length(algorithms));
    for i = 1:length(algorithms)
        semilogy(eval([algorithms{i} '_cg_curve']), 'Color', colors(i, :), 'LineWidth', 1);
        hold on
    end
    % title(['Convergence curve, dim=' num2str(dim)])
    title(['Convergence curve'])
    xlabel('Iteration')
    ylabel(['Best score ' num2str(Function_name)])
    axis tight
    grid on
    box on
    set(gca, 'color', 'none')
    % Add legend
    legend(algorithms, 'Location', 'Best');
end
