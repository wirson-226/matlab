%________________________________________________________________________________________________%
%  An improved snake optimizer (ISO) source codes demo 1.0                                       %
%                                                                                                %
%  Developed in MATLAB R2022a                                                                    %
%                                                                                                %
%  Author and programmer: Yunwei Zhu                                                             %
%                                                                                                %
%  e-Mail: gs.ywzhu19@gzu.edu.cn                                                                 %
%                                                                                                %
%  Main paper:                                                                                   %
%                                                                                                %
% ISO：An improved snake optimizer with multi-strategy enhancement for engineering optimizations %
% _______________________________________________________________________________________________%

% You can simply define your cost in a seperate file and load its handle to fobj 
% The initial parameters that you need are:
%__________________________________________________
% fobj = @YourCostFunction
% dim = number of your variables
% Max_iteration = maximum number of generations
% SearchAgents_no = number of search agents
% lb=[lb1;lb2;...;lbn] where lbn is the lower bound of variable n 
% ub=[ub1;ub2;...;ubn] where ubn is the upper bound of variable n
% If all the variables have equal lower bound you can just
% define lb and ub as two single number numbers
% To run ISO:  [Xfood, fval,Convergence_curve,Trajectories,fitness_history, position_history]=ISO(N,T,lb,ub,dim,fobj);
% _______________________________________________________________________________________________%

clear all 
clc

Function_name='F5'; % Name of the test function that can be from F1 to F23
N=30; % Number of search agents
T=200; % Maximum numbef of iterations
% Load details of the selected benchmark function
[lb,ub,dim,fobj]=Get_Functions_details(Function_name);

[Xfood, fval,Convergence_curve,Trajectories,fitness_history, position_history]=ISO(N,T,lb,ub,dim,fobj); %ISO
[Best_pos,Best_score,SO_curve]=SO(N,T,lb,ub,dim,fobj);%SO

figure('Position',[39         479        1740         267])
color1 = [205,92,92]/255;
color2 = [72,61,139]/255;
color3 = [0,206,209]/255;
color4 = [255,99,71]/255;
%Draw search space
subplot(1,5,1);
func_plot(Function_name);
title('Parameter space','FontName', 'Times New Roman')
xlabel('x_1','FontName', 'Times New Roman');
ylabel('x_2','FontName', 'Times New Roman');
zlabel([Function_name,'( x_1 , x_2 )'],'FontName', 'Times New Roman')
box on
axis tight
subplot(1,5,2);
semilogy(SO_curve,'Color',color2,'LineStyle','--','linewidth',2);
hold on
semilogy(Convergence_curve,'Color',color1,'LineStyle','-','linewidth',2);
title('Convergence curve', 'FontName', 'Times New Roman')
xlabel('Iteration#', 'FontName', 'Times New Roman');
ylabel('Best score obtained so far', 'FontName', 'Times New Roman');
box on
legend('SO','ISO', 'FontName', 'Times New Roman')
axis tight
subplot(1,5,3);
hold on
semilogy(Trajectories(1,:),'Color',color3,'linewidth',2);
title('Trajectory ','FontName', 'Times New Roman')
xlabel('Iteration#','FontName', 'Times New Roman')
box on
legend('ISO', 'FontName', 'Times New Roman')
axis tight
subplot(1,5,4);
hold on
a=mean(fitness_history);
semilogy(a,'Color',color4,'linewidth',2);
title('Average Fitness ','FontName', 'Times New Roman')
xlabel('Iteration#','FontName', 'Times New Roman')
box on
legend('ISO', 'FontName', 'Times New Roman')
axis tight
subplot(1,5,5);
hold on
for k1 = 1: size(position_history,1)
    for k2 = 1: size(position_history,2)
        plot(position_history(k1,k2,1),position_history(k1,k2,2),'o','markersize',5,'MarkerEdgeColor','k','markerfacecolor','#1e90ff');
    end
end
plot(Xfood(1),Xfood(2),'o','markersize',10,'MarkerEdgeColor','k','markerfacecolor','r','linewidth',1);
title('Search history (x1 and x2 only)','FontName', 'Times New Roman')
xlabel('x1','FontName', 'Times New Roman')
ylabel('x2','FontName', 'Times New Roman')
box on
axis tight
subplot(1,5,5);
hold on
func_plot1(Function_name)



