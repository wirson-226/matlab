
function [gbest_fitness gbest Fitness_Curve]= VPPSO(Function_name,max_iteration)


[lb,ub,dim,fobj]=Get_F_details(Function_name);

lb=lb.*ones(1,dim);
ub=ub.*ones(1,dim);

N=15; % Number of partilces of first swarm 
NT=N+15; % Number of partilces of second swarm 

w_Max = 0.9;
w_Min = 0.1;

c1 = 2;
c2 = 2;

X_min=lb;
X_max=ub;
V_max=0.1.*(ub-lb);
V_min=-V_max;
gbest_fitness=inf;

%% Intilization 
for i=1:N
     Position(i,:)=X_min+(X_max-X_min).*rand(1,dim);
      Velocity(i,:)=zeros(1,dim);

      fitness(i)=fobj(Position(i,:));
       Pbest(i,:)=Position(i,:);
            Pbest_finess(i)= fitness(i);
            
            if  Pbest_finess(i)<gbest_fitness
                gbest=Pbest(i,:);
                gbest_fitness=Pbest_finess(i);
               
            end
end

for t=1:max_iteration
   

      ww(t) =exp(-(2.5*t/max_iteration)^2.5); % Equ. 12
    for i=1:N


if rand<0.3 
% Equ. 13
Velocity(i,:)= abs(Velocity(i,:)).^(rand*ww(t))+rand* c1*(Pbest(i,:)-Position(i,:))+rand* c2*(gbest-Position(i,:)); 
end
       %% Velociy clamping
       index_Vmax = find(Velocity(i,:)> V_max);
         index_Vmin = find(Velocity(i,:)< V_min);
%         
        Velocity(i, index_Vmax) = V_max(index_Vmax);
          Velocity(i,  index_Vmin) = V_min( index_Vmin);
         Position(i,:)=Position(i,:)+Velocity(i,:);
         %% Boundry check
         index_Pos_ub=find(Position(i,:)> ub);
         index_Pos_lb=find(Position(i,:)< lb);
        Position(i, index_Pos_ub) = ub(index_Pos_ub);
          Position(i,   index_Pos_lb) =lb(  index_Pos_lb);
           
    end
    %% Second swarm
      for i=N+1:NT
          
            for j=1:dim
                
             %% Equ. 15
               CC=ww(t)*rand*abs(gbest(j))^ww(t);
         
                 if rand<0.5
          Position(i,j)=(gbest(j))+ CC;
                 else
                      Position(i,j)=(gbest(j))- CC;
                 end
              
           end
            %% Boundry check
         index_Pos_ub=find(Position(i,:)> ub);
         index_Pos_lb=find(Position(i,:)< lb);
        Position(i, index_Pos_ub) = ub(index_Pos_ub);
          Position(i,   index_Pos_lb) =lb(  index_Pos_lb);
        
        
      end
        
        
      
      for i=1:NT
        if i<=N
            %% Evalute fitness
            fitness(i)=fobj(Position(i,:));
            %% Update Pbest
            if fitness(i)< Pbest_finess(i)
                Pbest(i,:)=Position(i,:);
                Pbest_finess(i)=fitness(i);
                
                if Pbest_finess(i)<gbest_fitness
                    gbest=Pbest(i,:);
                    gbest_fitness=Pbest_finess(i);
                  
                end
            end
        else
             %% Evalute fitness
            fitness(i)=fobj(Position(i,:));
            if  fitness(i)<gbest_fitness
                    gbest=Position(i,:);
                    gbest_fitness=fitness(i);
                 
                  
                end
        end
    end
      
      Fitness_Curve(t)= gbest_fitness;
      
    end
    
    
    
   
    
    
    
    
    

