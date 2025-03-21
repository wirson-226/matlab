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

function [Xfood, fval,gbest_t,Trajectories,fitness_history, position_history] = ISO(N,T,lb,ub,dim,fobj)

%initial 
vec_flag=[1,-1];
Threshold=0.25;
Thresold2= 0.6;
C1=0.5;
C2=.05;
C3=2;
%  Strategy 1: Multi-strategy chaotic system(MSCS)
% % Initialization 
if length(lb)<2
X=lb+MSCS(N,dim)*(ub-lb);
else 
X=repmat(lb,N,1)+MSCS(N,dim).*repmat((ub-lb),N,1);%eq.(10)
end
for i=1:N
 fitness(i)=fobj(X(i,:)); 
end

Trajectories=zeros(N,T);
position_history=zeros(N,T,dim);
fitness_history=zeros(N,T);

[GYbest, gbest] = min(fitness);
Xfood = X(gbest,:);
%Diving the swarm into two equal groups males and females
Nm=round(N/2);%eq.(1)
Nf=N-Nm;
Xm=X(1:Nm,:);
Xf=X(Nm+1:N,:);
fitness_m=fitness(1:Nm);
fitness_f=fitness(Nm+1:N);
[fitnessBest_m, gbest1] = min(fitness_m);
Xbest_m = Xm(gbest1,:);
[fitnessBest_f, gbest2] = min(fitness_f);
Xbest_f = Xf(gbest2,:);
for t = 1:T
            Positions=[Xm;Xf];
    for i=1:size(Positions,1)
        position_history(i,t,:)=Positions(i,:);
        Trajectories(:,t)=Positions(:,1);
        fitness_history(i,t)=fobj(Positions(i,:));
    end

    Temp=exp(-((t)/T));  %eq.(2) 
    Q=C1*exp(((t-T)/(T)));%eq.(3)   
    if Q>1        Q=1;    end  
    % Exploration Phase (no Food)  
if Q<Threshold
    for i=1:Nm
        %Strategy 1: MSCS 
        lb_ap=lb(1)/t;
        ub_ap=ub(1)/t;
        z = rand(1, 1);
        z = sin(pi*(  z .* (1 - z)+ sin(pi * z))); %eq.(13)
        for j=1:1:dim
            rand_leader_index = floor(Nm*rand()+1);
            X_randm = Xm(rand_leader_index, :);
            flag_index = floor(2*rand()+1);
            Flag=vec_flag(flag_index);
            Am=exp(-fitness_m(rand_leader_index)/(fitness_m(i)+eps));
            Xnewm1(i,j)=X_randm(j)+Flag*C2*Am*((ub(1)-lb(1))*rand+lb(1));%eq.(4)   
        end     
        % Strategy 2: Anti-predator strategies(APS-m)
        Xnewm2(i,:)= X(i,:) -round(1+z).* (lb_ap+ rand(1,1) .* (ub_ap-lb_ap)); %eq.(16)
        Xnewm2(i,:) = max(Xnewm2(i,:),lb_ap);Xnewm2(i,:) = min(Xnewm2(i,:),ub_ap);
        if fobj(Xnewm2(i,:)) < fobj(Xnewm1(i,:))%eq.(17)
              Xnewm(i,:)=Xnewm2(i,:);%eq.(17)
        else 
            Xnewm(i,:)=Xnewm1(i,:);%eq.(17)
        end
        if(fobj(Xnewm(i,:))<fitness(i))%eq.(20)
            X(i,:) = Xnewm(i,:);%eq.(20)
            fitness(i) = fobj(Xnewm(i,:));
        end
    end
     % Strategy 2: Anti-predator strategies(APS-f)
    for i=1:Nf
        for j=1:1:dim
            rand_leader_index = floor(Nf*rand()+1);
            X_randf = Xf(rand_leader_index, :);
            flag_index = floor(2*rand()+1);
            Flag=vec_flag(flag_index);
            Af=exp(-fitness_f(rand_leader_index)/(fitness_f(i)+eps));
            Xnewf1(i,j)=X_randf(j)+Flag*C2*Af*((ub(1)-lb(1))*rand+lb(1));%eq.(4)
        end
        Xnewf2(i,:)=X(i,:)+rand(1,1) .* (Xbest_f-round(1+z).*X(i,:)); % Eq.(18) 
        Xnewf2(i,:) = max(Xnewf2(i,:),lb_ap);Xnewf2(i,:) = min(Xnewf2(i,:),ub_ap);
        if fobj(Xnewf2(i,:)) < fobj(Xnewf1(i,:))%Eq.(18) 
              Xnewf(i,:)=Xnewf2(i,:);%Eq.(18)
        else 
            Xnewf(i,:)=Xnewf1(i,:);%Eq.(18)
        end
        if(fobj(Xnewf(i,:))<fobj(Xbest_f))%eq.(20)
            X(i,:) = Xnewf(i,:);%eq.(20)
            fitness(i) = fobj(Xnewf(i,:));
        end
    end

else %Exploitation Phase (Food Exists)  
    if Temp>Thresold2  %hot  
        for i=1:Nm 
            flag_index = floor(2*rand()+1);
            Flag=vec_flag(flag_index);
            for j=1:1:dim
                Xnewm(i,j)=Xfood(j)+C3*Flag*Temp*rand*(Xfood(j)-Xm(i,j));%eq.(5)
            end
        end
        for i=1:Nf
            flag_index = floor(2*rand()+1);
            Flag=vec_flag(flag_index);
            for j=1:1:dim
                Xnewf(i,j)=Xfood(j)+Flag*C3*Temp*rand*(Xfood(j)-Xf(i,j));%eq.(5)
            end
        end
    else %cold
        if rand>0.6 %fight  
            for i=1:Nm
                for j=1:1:dim
                    FM=exp(-(fitnessBest_f)/(fitness_m(i)+eps));
                    Xnewm(i,j)=Xm(i,j) +C3*FM*rand*(Q*Xbest_f(j)-Xm(i,j));%eq.(6)      
                end
            end
            for i=1:Nf
                for j=1:1:dim
                    FF=exp(-(fitnessBest_m)/(fitness_f(i)+eps));
                    Xnewf(i,j)=Xf(i,j)+C3*FF*rand*(Q*Xbest_m(j)-Xf(i,j));%eq.(6)
                end
            end
        else%mating
            for i=1:Nm
                for j=1:1:dim
                    Mm=exp(-fitness_f(i)/(fitness_m(i)+eps));
                    Xnewm(i,j)=Xm(i,j) +C3*rand*Mm*(Q*Xf(i,j)-Xm(i,j));%eq.(7)
                end
            end
            for i=1:Nf
                for j=1:1:dim
                    Mf=exp(-fitness_m(i)/(fitness_f(i)+eps));
                    Xnewf(i,j)=Xf(i,j) +C3*rand*Mf*(Q*Xm(i,j)-Xf(i,j));%eq.(7)
                end
            end
            flag_index = floor(2*rand()+1);
            egg=vec_flag(flag_index);  
            if egg==1              
              %Strategy 3:Bidirectional population evolution dynamics (BPED)
              freq = 1/dim;
              X=[Xnewm;Xnewf];
              for i=1:N
                 FIT1(i)=fobj(X(i,:));
              end
                [PAIXU,I1]=sort(FIT1);
                a=ceil(N*0.8);%Pareto principle
                %%Mutation factor
                w=1/2*(sin(2*pi*freq*t+pi)*(t/T)+1);%eq.(23)
                for i=1:a
                  dx = randperm(a);             
                  q = dx(1);        k = dx(2); 
                  if q == i  
                        q  = dx(3);  
                  else
                    if k == i
                        k = dx(3);  
                    end
                  end   
                  for j=1:dim
                    Xgood_new(I1(i),j) = X(q,j) + w*(Xfood(j)-round(1+z)*X(k,j));%eq.(21)   
                  end
                   fit_gnew=fobj(Xgood_new(I1(i),:));
                   if fit_gnew< PAIXU(i)
                      X(I1(i),:)= Xgood_new(I1(i),j);%eq.(22)
                   end
                end
                % Population evolution dynamics(EPD)
                for i=a+1:N
                    for j=1:dim
                        z1=I1(i);
                        if rand<0.5
                          Xbad_new(z1,j)=Xfood(j)+sign(rand-0.50)*(lb_ap+rand*(ub_ap-lb_ap));%eq.(24)
                        else
                          Xbad_new(z1,j)=X(z1,j) - 2*sign(rand-0.50)*(lb(1)+rand*(ub(1)-lb(1)));%eq.(25)  
                        end
                    end
                    FIT2=fobj(Xbad_new(z1,:));
                        if FIT2<FIT1(z1)
                            X(z1,:)=Xbad_new(z1,:);
                        else
                            X(z1,:)=lb+rand*(ub-lb);
                        end
                end
                for i=1:Nm
                    Xnewm(i,:)=X(i,:);
                    Xnewf(i,:)=X(i+Nm,:);
                end
            end
        end
    end
end
    for j=1:Nm
         Flag4ub=Xnewm(j,:)>ub;
         Flag4lb=Xnewm(j,:)<lb;
        Xnewm(j,:)=(Xnewm(j,:).*(~(Flag4ub+Flag4lb)))+ub.*Flag4ub+lb.*Flag4lb;
        y = fobj(Xnewm(j,:));
        if y<fitness_m(j)
            fitness_m(j)=y;
            Xm(j,:)= Xnewm(j,:);
        end
    end
    [Ybest1,gbest1] = min(fitness_m);
    for j=1:Nf
         Flag4ub=Xnewf(j,:)>ub;
         Flag4lb=Xnewf(j,:)<lb;
        Xnewf(j,:)=(Xnewf(j,:).*(~(Flag4ub+Flag4lb)))+ub.*Flag4ub+lb.*Flag4lb;
        y = fobj(Xnewf(j,:));
        if y<fitness_f(j)
            fitness_f(j)=y;
            Xf(j,:)= Xnewf(j,:);
        end
    end
    [Ybest2,gbest2] = min(fitness_f);
    
    if Ybest1<fitnessBest_m
        Xbest_m = Xm(gbest1,:);
        fitnessBest_m=Ybest1;
    end
    if Ybest2<fitnessBest_f
        Xbest_f = Xf(gbest2,:);
        fitnessBest_f=Ybest2;
    end
    if Ybest1<Ybest2
        gbest_t(t)=min(Ybest1);
    else
        gbest_t(t)=min(Ybest2);
        
    end
    if fitnessBest_m<fitnessBest_f
        GYbest=fitnessBest_m;
        Xfood=Xbest_m;
    else
        GYbest=fitnessBest_f;
        Xfood=Xbest_f;
    end 
end
fval = GYbest;
end
%%  Strategy 1: MSCS
function z =MSCS(N,dim)
    z = rand(N, dim); 
    z = sin(pi*(  z .* (1 - z)+ sin(pi * z)));%eq.(13)
    z = abs(z);
end