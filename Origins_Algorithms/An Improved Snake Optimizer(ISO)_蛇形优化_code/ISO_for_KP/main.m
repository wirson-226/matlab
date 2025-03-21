clc;clear;close all
%%
% README:

% Author: Yunwei Zhu

% This is an optimized MATLAB version of the Knapsack Problem using ISO.

% Among them, KP8 is a recognized data set for the Knapsack Problem.

% This code is very easy to develop and deploy locally.
% You only need to run the main file and get the optimization results of the Knapsack Problem (KP8) using ISO. 
% The optimization results are the Maximum value.

% Very important! ! !
% When using code, please contact the author and cite the paper
% Email: gs.ywzhu19@gzu.edu.cn
% Reference: ISO: An improved snake optimizer with multi-strategy enhancement for engineering optimization

%%

pop_size=50;  
iter_max=500; 

Function_name= 'KP8'; 
[Xmin,Xmax,D,fobj]=Get_Functions_details(Function_name);

 % ISO
 [BEF_ISO,BEP_ISO,BestCost_ISO]=ISO(pop_size,iter_max,Xmin,Xmax,D,fobj);
 disp([' Maximum value  = ' num2str(-BEF_ISO)]);
   



