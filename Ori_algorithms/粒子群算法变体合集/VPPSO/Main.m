%_______________________________________________________________________________________
                    % Velocity Pausing Particle Swarm Optimization
% Paper: Velocity pausing particle swarm optimization: a novel variant
%        for global optimization
% Authors: Tareq M. Shami, Seyedali Mirjalili,  Yasser Al-Eryani, Khadija Daoudi,
%          Saadat Izadi, Laith Abualigah
%_____% Email: tariqshami2013@gmail.com (Tareq Alshami)
% Social Contacts:
% Researchgate: https://www.researchgate.net/profile/Tareq-M-Shami
% Linkedin: https://www.linkedin.com/in/tareq-al-shami-33162ab0/
% Reference: Shami, Tareq M., Seyedali Mirjalili, Yasser Al-Eryani, Khadija Daoudi,
% Saadat Izadi, and Laith Abualigah. "Velocity pausing particle swarm optimization:
% a novel variant for global optimization." Neural Computing and Applications (2023): 1-31.
%_______________________________________________________________________________
clc
clear all
close all
rand('state',sum(100.*clock));

Function_name='F1';

max_iteration=500;
r=30;
for r=1:r
[gbest_fitness gbest Fitness_Curve]= VPPSO(Function_name,max_iteration);
   Best_Fitness_VPPSO(r,:)=Fitness_Curve ;
r
end
Average_Fitness_VPPSO=mean( Best_Fitness_VPPSO,1);
Average_Best_score_VPPSO=Average_Fitness_VPPSO(max_iteration )
Best_score_VPPSO_last_iteration=Best_Fitness_VPPSO(:,max_iteration );
Standard_dev_VPPSO=std(Best_score_VPPSO_last_iteration);

plot(Fitness_Curve)
title('Convergence curve')
xlabel('Iteration');
ylabel('Averge Fitness');
  legend('VPPSO') 
