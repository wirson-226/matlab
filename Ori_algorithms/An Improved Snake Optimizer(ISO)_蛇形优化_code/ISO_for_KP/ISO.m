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

function [fval,Xfood,gbest_t] = ISO(N,T, lb,ub,dim,fobj)

vec_flag=[1,-1];
Threshold=0.25;
Thresold2= 0.6;
C1=0.5;
C2=.05;
C3=2;
 
if length(lb)<2
X=lb+MSCA(N,dim)*(ub-lb);
else 
X=repmat(lb,N,1)+MSCA(N,dim).*repmat((ub-lb),N,1);
end
for i=1:N
 fitness(i)=fobj(X(i,:)); 
end

[GYbest, gbest] = min(fitness);
Xfood = X(gbest,:);
Nm=round(N/2);
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
    Temp=exp(-((t)/T)); 
    Q=C1*exp(((t-T)/(T)));
    if Q>1        Q=1;    end  
if Q<Threshold

    for i=1:Nm

        lb_ap=lb(1)/t;
        ub_ap=ub(1)/t;
        z = rand(1, 1); 
        z = sin(pi*(  z .* (1 - z)+ sin(pi * z)));


        for j=1:1:dim
            rand_leader_index = floor(Nm*rand()+1);
            X_randm = Xm(rand_leader_index, :);
            flag_index = floor(2*rand()+1);
            Flag=vec_flag(flag_index);
            Am=exp(-fitness_m(rand_leader_index)/(fitness_m(i)+eps));
            Xnewm1(i,j)=X_randm(j)+Flag*C2*Am*((ub(1)-lb(1))*rand+lb(1));
        end     

        Xnewm2(i,:)= X(i,:) -round(1+z).* (lb_ap+ rand(1,1) .* (ub_ap-lb_ap)); 
        Xnewm2(i,:) = max(Xnewm2(i,:),lb_ap);Xnewm2(i,:) = min(Xnewm2(i,:),ub_ap);

        if fobj(Xnewm2(i,:)) < fobj(Xnewm1(i,:))
              Xnewm(i,:)=Xnewm2(i,:);
        else 
            Xnewm(i,:)=Xnewm1(i,:);
        end

        if(fobj(Xnewm(i,:))<fitness(i))
            X(i,:) = Xnewm(i,:);
            fitness(i) = fobj(Xnewm(i,:));
        end

    end

    for i=1:Nf
        for j=1:1:dim
            rand_leader_index = floor(Nf*rand()+1);
            X_randf = Xf(rand_leader_index, :);
            flag_index = floor(2*rand()+1);
            Flag=vec_flag(flag_index);
            Af=exp(-fitness_f(rand_leader_index)/(fitness_f(i)+eps));
            Xnewf1(i,j)=X_randf(j)+Flag*C2*Af*((ub(1)-lb(1))*rand+lb(1));
        end

        Xnewf2(i,:)=X(i,:)+rand(1,1) .* (Xbest_f-round(1+z).*X(i,:)); 
        Xnewf2(i,:) = max(Xnewf2(i,:),lb_ap);Xnewf2(i,:) = min(Xnewf2(i,:),ub_ap);
        if fobj(Xnewf2(i,:)) < fobj(Xnewf1(i,:))
              Xnewf(i,:)=Xnewf2(i,:);
        else 
            Xnewf(i,:)=Xnewf1(i,:);
        end
        
        if(fobj(Xnewf(i,:))<fobj(Xbest_f))
            X(i,:) = Xnewf(i,:);
            fitness(i) = fobj(Xnewf(i,:));
        end
    end

else 
    if Temp>Thresold2 
        for i=1:Nm 
            flag_index = floor(2*rand()+1);
            Flag=vec_flag(flag_index);
            for j=1:1:dim
                Xnewm(i,j)=Xfood(j)+C3*Flag*Temp*rand*(Xfood(j)-Xm(i,j));
            end
        end
        for i=1:Nf
            flag_index = floor(2*rand()+1);
            Flag=vec_flag(flag_index);
            for j=1:1:dim
                Xnewf(i,j)=Xfood(j)+Flag*C3*Temp*rand*(Xfood(j)-Xf(i,j));
            end
        end
    else 
        if rand>0.6 
            for i=1:Nm
                for j=1:1:dim
                    FM=exp(-(fitnessBest_f)/(fitness_m(i)+eps));
                    Xnewm(i,j)=Xm(i,j) +C3*FM*rand*(Q*Xbest_f(j)-Xm(i,j));     
                end
            end
            for i=1:Nf
                for j=1:1:dim
                    FF=exp(-(fitnessBest_m)/(fitness_f(i)+eps));
                    Xnewf(i,j)=Xf(i,j)+C3*FF*rand*(Q*Xbest_m(j)-Xf(i,j));
                end
            end
        else
            for i=1:Nm
                for j=1:1:dim
                    Mm=exp(-fitness_f(i)/(fitness_m(i)+eps));
                    Xnewm(i,j)=Xm(i,j) +C3*rand*Mm*(Q*Xf(i,j)-Xm(i,j));
                end
            end
            for i=1:Nf
                for j=1:1:dim
                    Mf=exp(-fitness_m(i)/(fitness_f(i)+eps));
                    Xnewf(i,j)=Xf(i,j) +C3*rand*Mf*(Q*Xm(i,j)-Xf(i,j));
                end
            end
            flag_index = floor(2*rand()+1);
            egg=vec_flag(flag_index);
            if egg==1  
              freq = 1/dim;
              X=[Xnewm;Xnewf];
              for i=1:N
                 FIT1(i)=fobj(X(i,:));
              end
                [PAIXU,I1]=sort(FIT1);
                a=ceil(N*0.8);
                w=1/2*(sin(2*pi*freq*t+pi)*(t/T)+1);
                  
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
                    Xnew_son(I1(i),j) = X(q,j) + w*(Xfood(j)-round(1+z)*X(k,j));    
                  end
                   fit_mson=fobj(Xnew_son(I1(i),:));
                   if fit_mson< PAIXU(i)
                      X(I1(i),:)= Xnew_son(I1(i),j);
                   end
                end
                for i=a+1:N
                    for j=1:dim
                        z1=I1(i);
                        if rand<0.5
                          Xnew1(z1,j)=Xfood(j)+sign(rand-0.50)*(lb_ap+rand*(ub_ap-lb_ap));
                        else
                          Xnew1(z1,j)=X(z1,j) - 2*sign(rand-0.50)*(lb(1)+rand*(ub(1)-lb(1)));  
                        end
                    end
                    FIT2=fobj(Xnew1(z1,:));
                        if FIT2<FIT1(z1)
                            X(z1,:)=Xnew1(z1,:);
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
function z =MSCA(N,dim)
    z = rand(N, dim);
    z = sin(pi*(  z .* (1 - z)+ sin(pi * z)));
    z = abs(z);
end