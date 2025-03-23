
clear
clc
close all
pop_size=50;   
max_iter=500;  

run = 10;
RESULT=[];   %统计标准差，平均值，最优值等结果

F = [1 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30];
variables_no = 30; %2, 10, 30, 50, 100
disp(['Currently calculating the CEC2017 function set with a dimensionality of ', num2str(variables_no), ''])

for func_num = 1:length(F)   
    % Display the comprehensive results
    disp(['F',num2str(F(func_num)),' Function calculation result：'])
    num=F(func_num);
    [lower_bound,upper_bound,variables_no,fobj]=Get_Functions_cec2017(num,variables_no);  % [lb, ub, D, y]: lower bound, upper bound, dimensionality, and objective function expression
    resu = [];  % Calculate statistics: standard deviation, mean, optimal value, and other results
    
    %% Run the DOA algorithm for "run" times
  for nrun=1:run
        [final,position]=DOA(pop_size,max_iter,lower_bound,upper_bound,variables_no,fobj);
        final_main(nrun)=final;
        z1(nrun) =  final;
  end
    zz = [min(final_main);mean(final_main);std(final_main);median(final_main);max(final_main)];
    resu = [resu,zz];
    disp(['DOA: Optimal value: ', num2str(zz(1)), ', Mean: ', num2str(zz(2)), ', Standard deviation: ', num2str(zz(3)), ', Median: ', num2str(zz(4)), ' Worst value: ', num2str(zz(5)), ]);
end
    RESULT = [RESULT;resu];   %Calculate and summarize results: standard deviation, mean, optimal value, and other metrics

