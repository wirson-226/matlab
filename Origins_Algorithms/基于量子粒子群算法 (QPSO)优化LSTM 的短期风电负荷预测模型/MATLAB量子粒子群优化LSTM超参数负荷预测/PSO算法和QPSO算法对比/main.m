clc;close all
%%
Function_name='F2'; % Name of the test function that can be from F1 to F23 (Table 1,2,3 in the paper) 设定适应度函数
[lb,ub,dim,fobj]=Get_Functions_details(Function_name);  %设定边界以及优化函数

N=20;
M=100000;
[xm1,trace1]=pso(N,M,dim,lb,ub,fobj);
[xm2,trace2]=qpso(N,M,dim,lb,ub,fobj);

figure('Position',[269   240   660   290])
%Draw search space
subplot(1,2,1);
func_plot(Function_name);
title('Parameter space')
xlabel('x_1');
ylabel('x_2');
zlabel([Function_name,'( x_1 , x_2 )'])

%Draw objective space
subplot(1,2,2);
plot(trace1,'Color','b','linewidth',1.5)
hold on
plot(trace2,'Color','r','linewidth',1.5)
title('Objective space')
xlabel('Iteration');
ylabel('Best score obtained so far');

axis tight
grid on
box on
legend('PSO','QPSO')

%% 取对数 更方便看
figure
plot(log10(trace1))
hold on
plot(log10(trace2))
legend('PSO','QPSO')
title('PSO VS QPSO')
xlabel('iteration/M')
ylabel('fitness value(log10)')