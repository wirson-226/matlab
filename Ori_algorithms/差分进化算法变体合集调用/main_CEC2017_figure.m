clc
clear all
close all
addpath("���׺���")
%% ������ʼ������
algorithms = {'DE','ADE','SaDE','JADE','IMODE','CJADE','SHADE','LSHADE','ALSHADE','EBOwithCMA','LSHADE_EpSin','LSHADE_SPACMA','mLSHADE_SPACMA'};

nPop = 100;                  %��Ⱥ��
Max_iter = 1000;             %����������
dim = 10;                   %��ѡ 10��30��50��100

for j = [1, 3:4]
    %% ѡ����
    Function_name = j; % �������� 1 - 30
    [lb, ub, dim, fobj] = Get_CEC2017_details(Function_name,dim);

    %% DE�㷨�������㷨��
    tic
    [DE_Best_score, DE_Best_pos, DE_cg_curve] = DE(nPop, Max_iter, lb, ub, dim, fobj);
    toc
    display(['The best optimal value of the objective function found by DE for F' num2str(Function_name), ' is: ', num2str(DE_Best_score)]);
    fprintf('Best solution obtained by DE: %s\n', num2str(DE_Best_pos, '%e  '));
           
    %% ADE�㷨�������㷨��
    tic
    [ADE_Best_score, ADE_Best_pos, ADE_cg_curve] = ADE(nPop, Max_iter, lb, ub, dim, fobj);
    toc
    display(['The best optimal value of the objective function found by ADE for F' num2str(Function_name), ' is: ', num2str(ADE_Best_score)]);
    fprintf('Best solution obtained by ADE: %s\n', num2str(ADE_Best_pos, '%e  '));

    %% SaDE�㷨�������㷨��
    tic
    [SaDE_Best_score, SaDE_Best_pos, SaDE_cg_curve] = SaDE(nPop, Max_iter, lb, ub, dim, fobj);
    toc
    display(['The best optimal value of the objective function found by SaDE for F' num2str(Function_name), ' is: ', num2str(SaDE_Best_score)]);
    fprintf('Best solution obtained by SaDE: %s\n', num2str(SaDE_Best_pos, '%e  '));

    %% JADE�㷨�������㷨��
    tic
    [JADE_Best_score, JADE_Best_pos, JADE_cg_curve] = JADE(nPop, Max_iter, lb, ub, dim, fobj);
    toc
    display(['The best optimal value of the objective function found by JADE for F' num2str(Function_name), ' is: ', num2str(JADE_Best_score)]);
    fprintf('Best solution obtained by JADE: %s\n', num2str(JADE_Best_pos, '%e  '));

    %% IMODE�㷨�������㷨��
    tic
    [IMODE_Best_score, IMODE_Best_pos, IMODE_cg_curve] = IMODE(nPop, Max_iter, lb, ub, dim, fobj);
    toc
    display(['The best optimal value of the objective function found by IMODE for F' num2str(Function_name), ' is: ', num2str(IMODE_Best_score)]);
    fprintf('Best solution obtained by IMODE: %s\n', num2str(IMODE_Best_pos, '%e  '));


    %% CJADE�㷨�������㷨��
    tic
    [CJADE_Best_score, CJADE_Best_pos, CJADE_cg_curve] = CJADE(nPop, Max_iter, lb, ub, dim, fobj);
    toc
    display(['The best optimal value of the objective function found by CJADE for F' num2str(Function_name), ' is: ', num2str(CJADE_Best_score)]);
    fprintf('Best solution obtained by CJADE: %s\n', num2str(CJADE_Best_pos, '%e  '));

    %% SHADE�㷨�������㷨��
    tic
    [SHADE_Best_score, SHADE_Best_pos, SHADE_cg_curve] = SHADE(nPop, Max_iter, lb, ub, dim, fobj);
    toc
    display(['The best optimal value of the objective function found by SHADE for F' num2str(Function_name), ' is: ', num2str(SHADE_Best_score)]);
    fprintf('Best solution obtained by SHADE: %s\n', num2str(SHADE_Best_pos, '%e  '));

    %% LSHADE�㷨�������㷨��
    tic
    [LSHADE_Best_score, LSHADE_Best_pos, LSHADE_cg_curve] = LSHADE(nPop, Max_iter, lb, ub, dim, fobj);
    toc
    display(['The best optimal value of the objective function found by LSHADE for F' num2str(Function_name), ' is: ', num2str(LSHADE_Best_score)]);
    fprintf('Best solution obtained by LSHADE: %s\n', num2str(LSHADE_Best_pos, '%e  '));

    %% ALSHADE�㷨�������㷨��
    tic
    [ALSHADE_Best_score, ALSHADE_Best_pos, ALSHADE_cg_curve] = ALSHADE(nPop, Max_iter, lb, ub, dim, fobj);
    toc
    display(['The best optimal value of the objective function found by ALSHADE for F' num2str(Function_name), ' is: ', num2str(ALSHADE_Best_score)]);
    fprintf('Best solution obtained by ALSHADE: %s\n', num2str(ALSHADE_Best_pos, '%e  '));

    %% EBOwithCMA�㷨�������㷨��
    tic
    [EBOwithCMA_Best_score, EBOwithCMA_Best_pos, EBOwithCMA_cg_curve] = EBOwithCMA(nPop, Max_iter, lb, ub, dim, fobj);
    toc
    display(['The best optimal value of the objective function found by EBOwithCMA for F' num2str(Function_name), ' is: ', num2str(EBOwithCMA_Best_score)]);
    fprintf('Best solution obtained by EBOwithCMA: %s\n', num2str(EBOwithCMA_Best_pos, '%e  '));

    %% LSHADE_EpSin�㷨�������㷨��
    tic
    [LSHADE_EpSin_Best_score, LSHADE_EpSin_Best_pos, LSHADE_EpSin_cg_curve] = LSHADE_EpSin(nPop, Max_iter, lb, ub, dim, fobj);
    toc
    display(['The best optimal value of the objective function found by LSHADE_EpSin for F' num2str(Function_name), ' is: ', num2str(LSHADE_EpSin_Best_score)]);
    fprintf('Best solution obtained by LSHADE_EpSin: %s\n', num2str(LSHADE_EpSin_Best_pos, '%e  '));

    %% LSHADE_SPACMA�㷨�������㷨��
    tic
    [LSHADE_SPACMA_Best_score, LSHADE_SPACMA_Best_pos, LSHADE_SPACMA_cg_curve] = LSHADE_SPACMA(nPop, Max_iter, lb, ub, dim, fobj);
    toc
    display(['The best optimal value of the objective function found by LSHADE_SPACMA for F' num2str(Function_name), ' is: ', num2str(LSHADE_SPACMA_Best_score)]);
    fprintf('Best solution obtained by LSHADE_SPACMA: %s\n', num2str(LSHADE_SPACMA_Best_pos, '%e  '));
    
    %% mLSHADE_SPACMA�㷨�������㷨��
    tic
    [mLSHADE_SPACMA_Best_score, mLSHADE_SPACMA_Best_pos, mLSHADE_SPACMA_cg_curve] = mLSHADE_SPACMA(nPop, Max_iter, lb, ub, dim, fobj);
    toc
    display(['The best optimal value of the objective function found by mLSHADE_SPACMA for F' num2str(Function_name), ' is: ', num2str(mLSHADE_SPACMA_Best_score)]);
    fprintf('Best solution obtained by mLSHADE_SPACMA: %s\n', num2str(mLSHADE_SPACMA_Best_pos, '%e  '));

    %% plot��ͼ
    figure;
    colors = lines(length(algorithms));
    for i = 1:length(algorithms)
        semilogy(eval([algorithms{i} '_cg_curve']), 'Color', colors(i, :), 'Linewidth', 1);
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

    %     %% д������
    % % Create an Excel file to write the results
    % filename = '.xlsx';
    % % Define the function name
    % functionIndex = ['F' num2str(Function_name)];
    % results{end+1, 1} = functionIndex;
    % for i = 1:length(algorithms)
    %     results{end, i+1} = eval([algorithms{i} '_Best_score']);
    % end
    % xlswrite(filename, results);


end