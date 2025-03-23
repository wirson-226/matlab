clc
clear all
close all

%% ������ʼ������
algorithms = {'GWO','Clb_GWO','HBBOG','SOGWO','VAGWO'};
nPop = 30;                  %��Ⱥ��
Max_iter = 500;             %����������
dim = 50;                   %��ѡ 10��30��50��100

for j = 1:3
    %% ѡ����
    % Function_name = ['F' num2str(j)]; % �������� 1 - 30
    Function_name=j;
    [lb, ub, dim, fobj] = Get_CEC2017_details(Function_name,dim);
        %% GWO�㷨�������㷨��
    tic
    [GWO_Best_score, GWO_Best_pos, GWO_cg_curve] = GWOCS(nPop, Max_iter, lb, ub, dim, fobj);
    toc
    display(['The best optimal value of the objective function found by GWO for F' num2str(Function_name), ' is: ', num2str(GWO_Best_score)]);
    fprintf('Best solution obtained by GWO: %s\n', num2str(GWO_Best_pos, '%e  '));

    %% Clb_GWO�㷨�������㷨��
    tic
    [Clb_GWO_Best_score, Clb_GWO_Best_pos, Clb_GWO_cg_curve] = Clb_GWO(nPop, Max_iter, lb, ub, dim, fobj);
    toc
    display(['The best optimal value of the objective function found by Clb_GWO for F' num2str(Function_name), ' is: ', num2str(Clb_GWO_Best_score)]);
    fprintf('Best solution obtained by Clb_GWO: %s\n', num2str(Clb_GWO_Best_pos, '%e  '));
    
       %% HBBOG�㷨�������㷨��
    tic
    [HBBOG_Best_score, HBBOG_Best_pos, HBBOG_cg_curve] = HBBOG(nPop, Max_iter, lb, ub, dim, fobj);
    toc
    display(['The best optimal value of the objective function found by HBBOG for F' num2str(Function_name), ' is: ', num2str(HBBOG_Best_score)]);
    fprintf('Best solution obtained by HBBOG: %s\n', num2str(HBBOG_Best_pos, '%e  '));

           %% SOGWO�㷨�������㷨��
    tic
    [SOGWO_Best_score, SOGWO_Best_pos, SOGWO_cg_curve] = SOGWO(nPop, Max_iter, lb, ub, dim, fobj);
    toc
    display(['The best optimal value of the objective function found by SOGWO for F' num2str(Function_name), ' is: ', num2str(SOGWO_Best_score)]);
    fprintf('Best solution obtained by SOGWO: %s\n', num2str(SOGWO_Best_pos, '%e  '));
               %% VAGWO�㷨�������㷨��
    tic
    [VAGWO_Best_score, VAGWO_Best_pos, VAGWO_cg_curve] = VAGWO(nPop, Max_iter, lb, ub, dim, fobj);
    toc
    display(['The best optimal value of the objective function found by VAGWO for F' num2str(Function_name), ' is: ', num2str(VAGWO_Best_score)]);
    fprintf('Best solution obtained by VAGWO: %s\n', num2str(VAGWO_Best_pos, '%e  '));
    %% plot��ͼ
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
