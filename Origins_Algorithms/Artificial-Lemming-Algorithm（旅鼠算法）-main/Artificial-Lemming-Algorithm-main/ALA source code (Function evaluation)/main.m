% Artificial lemming algorithm: A novel bionic meta-heuristic technique for solving real-world engineering optimization problems
%                                                                                                     
%  Developed in MATLAB R2024a                                                                 
%                                                                                                     
%  Author :Yaning Xiao, Hao Cui, Ruba Abu Khurma, Pedro A. Castillo                                           
%                                                                                                     
%         e-Mail: xiaoyn2023@mail.sustech.edu.cn; cuihao@nefu.edu.cn; Rubaabukhurma82@gmail.com; pacv@ugr.es                                                                                                                                                                                          
%_______________________________________________________________________________________________
% You can simply define your cost function in a seperate file and load its handle to fobj 
% The initial parameters that you need are:
%__________________________________________
% fobj = @YourCostFunction
% dim = number of your variables
% MaxFEs = number of function evaluations
% N = number of search agents
% lb=[lb1,lb2,...,lbn] where lbn is the lower bound of variable n
% ub=[ub1,ub2,...,ubn] where ubn is the upper bound of variable n
% If all the variables have equal lower bound you can just
% define lb and ub as two single numbers
% 2024-05-16
%______________________________________________________________________________________________

close all; clear all; clc

N = 30;  
Function_name = 'F1'; 
[lb, ub, dim, fobj] = CEC2017(Function_name); % CEC2017

MaxFEs = 30000;
[ala_score, ala_pos, ala_curve] = ALA(N, MaxFEs, lb, ub, dim, fobj);

disp(['The best fitness of ALA is: ', num2str(ala_score)]);

% === 绘图优化 ===
fig = figure('Name','ALA Result','Units','centimeters','Position',[5, 5, 40, 30]);
tiledlayout(1,2,'TileSpacing','compact','Padding','compact');

% --- 子图1: 函数形状图 ---
ax1 = nexttile;
func_plot_cec2017(Function_name);
title([Function_name], 'Interpreter','latex');
xlabel('$x_1$', 'Interpreter','latex');
ylabel('$x_2$', 'Interpreter','latex');
zlabel(['$', Function_name, '(x_1, x_2)$'], 'Interpreter','latex');
grid on; box on;

% --- 子图2: 收敛曲线 ---
ax2 = nexttile;
semilogy(ala_curve, 'r', 'LineWidth', 2);
axis tight;
title(['ALA Convergence on ', Function_name], 'Interpreter','latex');
xlabel('Iteration', 'Interpreter','latex');
ylabel('Best Score So Far', 'Interpreter','latex');
legend('ALA', 'Location','northeast', 'Interpreter','latex', 'Box','off', 'FontSize', 30);
grid on; box on;

% === 格式统一 ===
set([ax1, ax2], 'FontName','Times New Roman', 'FontSize', 20, 'LineWidth', 1);

% 可选导出高清图像
print(fig, ['ALA_', Function_name, '_Result'], '-dpng', '-r1000');


