function [Alpha_score,Alpha_pos,Convergence_curve]=GWOCS(SearchAgents_no,Max_iter,lb,ub,dim,fobj)
% initialize alpha, beta, and delta_pos
% dim=2; ub=x;lb=1;Max_iter=10;
% SearchAgents_no=10;
% p=initialization(SearchAgents_no,dim,ub,lb);
Alpha_pos=zeros(1,dim);
% Alpha_pos=zeros,dim);
Alpha_score=inf; %change this to -inf for maximization problems
Beta_pos=zeros(1,dim);
Beta_score=inf; %change this to -inf for maximization problems
Delta_pos=zeros(1,dim);
Delta_score=inf; %change this to -inf for maximization problems
%Initialize the positions of search agents
Positions=initialization(SearchAgents_no,dim,ub,lb);
Positions=sort(Positions);
oldPositions=Positions;
Convergence_curve=zeros(1,Max_iter);
l=0;% Loop counter
% Main loop
while l<Max_iter
    for i=1:size(Positions,1)  
        
       % Return back the search agents that go beyond the boundaries of the search space
        Flag4ub=Positions(i,:)>ub;
        Flag4lb=Positions(i,:)<lb;
        Positions(i,:)=(Positions(i,:).*(~(Flag4ub+Flag4lb)))+ub.*Flag4ub+lb.*Flag4lb;               
%         Positions(i,:)=sort(Positions(i,:));
        % Calculate objective function for each search agent
%        [Positions(i,:), fitness(i)]=objf(Positions(i,:)',measuredPos,measuredPos,BlindDeviceID,actualRefLocs,refDeviceID,Range,x);
        fitness(i)=fobj(Positions(i,:));
        % Update Alpha, Beta, and Delta
        if fitness(i)<Alpha_score 
            Alpha_score=fitness(i); % Update alpha
            Alpha_pos=Positions(i,:);
        end
        
        if fitness(i)>Alpha_score && fitness(i)<Beta_score 
            Beta_score=fitness(i); % Update beta
            Beta_pos=Positions(i,:);
            
        end
        
        if fitness(i)>Alpha_score && fitness(i)>Beta_score && fitness(i)<Delta_score 
            Delta_score=fitness(i); % Update delta
            Delta_pos=Positions(i,:);
        end
    end
    
    
    a=2-l*((2)/Max_iter); % a decreases linearly fron 2 to 0
    
    % Update the Position of search agents including omegas
    for i=1:size(Positions,1)
        for j=1:size(Positions,2)     
                   
            r1=rand(); % r1 is a random number in [0,1]
            r2=rand(); % r2 is a random number in [0,1]
            
            A1=2*a*r1-a; % Equation (3.3)
            C1=2*r2; % Equation (3.4)
            
            D_alpha=abs(C1*Alpha_pos(j)-Positions(i,j)); % Equation (3.5)-part 1
            X1(i,j)=Alpha_pos(j)-A1*D_alpha; % Equation (3.6)-part 1
                       
            r1=rand();
            r2=rand();
            
            A2=2*a*r1-a; % Equation (3.3)
            C2=2*r2; % Equation (3.4)
            
            D_beta=abs(C2*Beta_pos(j)-Positions(i,j)); % Equation (3.5)-part 2
            X2(i,j)=Beta_pos(j)-A2*D_beta; % Equation (3.6)-part 2       
            
            r1=rand();
            r2=rand(); 
            
            A3=2*a*r1-a; % Equation (3.3)
            C3=2*r2; % Equation (3.4)
            
            D_delta=abs(C3*Delta_pos(j)-Positions(i,j)); % Equation (3.5)-part 3
            X3(i,j)=Delta_pos(j)-A3*D_delta; % Equation (3.5)-part 3             
            
%             Positions(i,j)=(X1+X2+X3)/3;% Equation (3.7)
            
        end
    end
    %% Cuckoo Search integrated here and take control from GWO
  
    % 
    %  the key group parameters in GWO are updated by cuckoo search's
    %  position updation formula 
    %    
    [~,index]=min(fitness);
    best=Positions(index,:);
    X1=get_cuckoos(X1,best,lb,ub); 
    X2=get_cuckoos(X2,best,lb,ub);
    X3=get_cuckoos(X3,best,lb,ub);
    %% control is sent back to GWO
    Positions=(X1+X2+X3)/3;% Equation (3.7)
%     Positions=sort(Positions);
    l=l+1;    
    Convergence_curve(l)=Alpha_score;
end
end

function Positions=initialization(SearchAgents_no,dim,ub,lb)
Boundary_no= size(ub,2); % numnber of boundaries
% If the boundaries of all variables are equal and user enter a signle
% number for both ub and lb
if Boundary_no==1
    Positions=rand(SearchAgents_no,dim).*(ub-lb)+lb;
end
% If each variable has a different lb and ub
if Boundary_no>1
    for i=1:dim
        ub_i=ub(i);
        lb_i=lb(i);
        Positions(:,i)=rand(SearchAgents_no,1).*(ub_i-lb_i)+lb_i;
    end
end
end

%% Get cuckoos by ramdom walk
function nest=get_cuckoos(nest,best,Lb,Ub)
% Levy flights
n=size(nest,1);
% Levy exponent and coefficient
% For details, see equation (2.21), Page 16 (chapter 2) of the book
% X. S. Yang, Nature-Inspired Metaheuristic Algorithms, 2nd Edition, Luniver Press, (2010).
beta=3/2;
sigma=(gamma(1+beta)*sin(pi*beta/2)/(gamma((1+beta)/2)*beta*2^((beta-1)/2)))^(1/beta);
for j=1:n,
    s=nest(j,:);
    % This is a simple way of implementing Levy flights
    % For standard random walks, use step=1;
    %% Levy flights by Mantegna's algorithm
    u=randn(size(s))*sigma;
    v=randn(size(s));
    step=u./abs(v).^(1/beta);
  
    % In the next equation, the difference factor (s-best) means that 
    % when the solution is the best solution, it remains unchanged.     
    stepsize=0.001*step.*(s-best);
    % Here the factor 0.01 comes from the fact that L/100 should the typical
    % step size of walks/flights where L is the typical lenghtscale; 
    % otherwise, Levy flights may become too aggresive/efficient, 
    % which makes new solutions (even) jump out side of the design domain 
    % (and thus wasting evaluations).
    % Now the actual random walks or flights
    s=s+stepsize.*randn(size(s));
   % Apply simple bounds/limits
   nest(j,:)=simplebounds(s,Lb,Ub);
end
end
% Application of simple constraints
function s=simplebounds(s,lb,ub)
Flag4ub=s>ub;
Flag4lb=s<lb;
s=s.*(~(Flag4ub+Flag4lb))+ub.*Flag4ub+lb.*Flag4lb;
end
