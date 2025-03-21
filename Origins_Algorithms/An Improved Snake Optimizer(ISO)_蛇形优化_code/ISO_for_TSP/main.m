clc;clear;close all
%%
% README:

% Author: Yunwei Zhu

% This is an optimized MATLAB version of the traveling salesman problem using ISO.

% Among them, att48 is a recognized data set for the traveling salesman problem.

% This code is very easy to develop and deploy locally.
% You only need to run the main file and get the optimization results of the traveling salesman problem (att48) using ISO. 
% The optimization results are the minimum path cost.

% Very important! ! !
% When using code, please contact the author and cite the paper
% Email: gs.ywzhu19@gzu.edu.cn
% Reference: ISO: An improved snake optimizer with multi-strategy enhancement for engineering optimization

%%

pop_size=50;   
iter_max=500; 
model=CreateModel();    
fobj=@(tour) TourLength(tour,model); 
nVar=model.n;                
VarSize=[1 nVar];      
Xmin=-5 ;   
Xmax=5 ;   
D=nVar;
[BEF_ISO,BEP_ISO,BestCost_ISO]=ISO(pop_size,iter_max,Xmin,Xmax,D,fobj);
disp([' Minimum path cost = ' num2str(BEF_ISO)]);