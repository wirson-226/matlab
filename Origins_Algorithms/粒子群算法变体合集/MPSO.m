%================================================
% MPSO: A Modified Particle Swarm Optimization Using Adaptive Strategy
% % Authors: Hao Liu, XueWei Zhang, LiangPing Tu
% Journal: Expert System With Applications
% Email: haoliu@ustl.edu.cn,    liuhustl@sina.cn
%================================================
function [gbestValue,Gbest, Fbest]=MPSO(N,T_Max,LB,UB,dim,fobj)
Vmax0=0.5*(UB-LB);
%% Inertial Weight
% w=0.72984;
w_max=0.9;
w_min=0.4;
Fbest=[];   x=rand;
%% Learning Factor
c1=2;%1.49618;
c2=c1;
%% Initialize Population
Xmax=repmat(UB,N,1);
Xmin=repmat(LB,N,1);
X=Xmin+(Xmax-Xmin).*rand(N,dim);
Vmax=repmat(Vmax0,N,1);
V=-Vmax+2*Vmax.*rand(N,dim);
%% Evaluation of particles.
fX(N)=inf;
for i=1:N
    fX(i)=fobj(X(i,:));
end
FEs=N;
%% Initialize Pbest and Gbest
Pbest=X;   
fPbest=fX;
[gbestValue, gbestIndex]=min(fPbest); 
Gbest=Pbest(gbestIndex,:); 
Fbest=[Fbest gbestValue];   
%% Iteration
for t=2:T_Max
    %% Update Position X and Velocity V and check the responding bounds
    x=4*x*(1-x);
    w=x*w_min+(w_max-w_min)*t/T_Max;
    u=randperm(N,2);
    for i=1:N
        [U,index]=min(fPbest(u));
        if min(fPbest(u))<fPbest(i)
            Ubest(i,:)=Pbest(index,:);
        else
            Ubest(i,:)=Pbest(i,:);
        end
    end
    V=w*V+c1*rand(N,dim).*(Ubest-X)+c2*rand(N,dim).*(repmat(mean(Pbest),N,1)-X);%(repmat(mean(Pbest),N,1)-X);
    V=max(-Vmax,min(Vmax,V));
    for i=1:N
        if exp(fX(i))/exp(mean(fX))>rand
            X(i,:)=w*X(i,:)+(1-w)*V(i,:)+Gbest;
        else
            X(i,:)=X(i,:)+V(i,:);
        end
    end
    X=max(Xmin,min(Xmax,X));
    %% ==捕食者捕食种群中的老弱病残(竞争淘汰机制或称末位淘汰机制)==
    for i=1:N
        fX(i)=fobj(X(i,:));
    end
    [worst,index]=max(fX);
    z=[1:index-1,index+1:N];
    d=randperm(length(z),2);
    NewX=Gbest+rand*(Pbest(z(d(2)),:)-Pbest(z(d(1)),:));
    NewX=max(LB,min(UB,NewX));
    fNewX=fobj(NewX);
    if fNewX<worst
        X(index,:)=NewX;
        fX(index)=fNewX;
    end
    %% ==Evaluate Fitness and Change Pbest
    for i=1:N
        if fX(i) < fPbest(i)
            Pbest(i,:)=X(i,:);
            fPbest(i)=fX(i);
        end
        if fPbest(i)<gbestValue
            Gbest=Pbest(i,:);
            gbestValue=fPbest(i);
        end
    end
    FEs = FEs + N;
    %% Record GbestValue so far for plotting
     if rem(t,4)==0
         Fbest=[Fbest gbestValue];
     end 
     % cg_curve(t)=Gbest;
end % end iteration
end % end function    