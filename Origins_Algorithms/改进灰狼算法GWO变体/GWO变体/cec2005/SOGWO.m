% Opposition based Grey Wolf Optimizer
function [Alpha_score,Alpha_pos,Convergence_curve]=SOGWO(SearchAgents_no,Max_iter,lb,ub,dim,fobj)

% initialize alpha, beta, and delta_pos
Alpha_pos=zeros(1,dim);
Alpha_score=inf; %change this to -inf for maximization problems

Beta_pos=zeros(1,dim);
Beta_score=inf; %change this to -inf for maximization problems

Delta_pos=zeros(1,dim);
Delta_score=inf; %change this to -inf for maximization problems

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%initializing boundary for opposition
Boundary_no= size(ub,2); % numnber of boundaries

% If the boundaries of all variables are equal and user enter a signle
% number for both ub and lb
if Boundary_no==1
    for i=1:dim
        upper(1,i)=ub;
        lower(1,i)=lb;
    end
    % If each variable has a different lb and ub
else
    for i=1:dim
        upper(1,i)=ub(i);
        lower(1,i)=lb(i);
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Initialize the positions of search agents
Positions=initialization(SearchAgents_no,dim,ub,lb);

Convergence_curve=zeros(1,Max_iter);

l=0;% Loop counter

fitness=zeros(size(Positions(:,1)));
% Main loop
while l<Max_iter
    for i=1:size(Positions,1)
        
        % Return back the search agents that go beyond the boundaries of the search space
        Flag4ub=Positions(i,:)>ub;
        Flag4lb=Positions(i,:)<lb;
        Positions(i,:)=(Positions(i,:).*(~(Flag4ub+Flag4lb)))+ub.*Flag4ub+lb.*Flag4lb;
        
        % Calculate objective function for each search agent
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
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %updating boundary for opposition after every iteration
        for x=1:size(Positions,1)
            for y=1:size(Positions,2)
                if upper(1,y)<Positions(x,y)
                    upper(1,y)=Positions(x,y);
                end
                if lower(1,y)>Positions(x,y)
                    lower(1,y)=Positions(x,y);
                end
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    end
    
    
    a=2-l*((2)/Max_iter); % a decreases linearly fron 2 to 0
    
    % Oppose the least fitness elements
    threshold=a;
    Positions=corOppose(Positions,fitness,ub,lb,upper,lower,dim,threshold);
    
    % Update the Position of search agents including omegas
    for i=1:size(Positions,1)
        for j=1:size(Positions,2)
            
            r1=rand(); % r1 is a random number in [0,1]
            r2=rand(); % r2 is a random number in [0,1]
            
            A1=2*a*r1-a; % Equation (3.3)
            C1=2*r2; % Equation (3.4)
            
            D_alpha=abs(C1*Alpha_pos(j)-Positions(i,j)); % Equation (3.5)-part 1
            X1=Alpha_pos(j)-A1*D_alpha; % Equation (3.6)-part 1
            
            r1=rand();
            r2=rand();
            
            A2=2*a*r1-a; % Equation (3.3)
            C2=2*r2; % Equation (3.4)
            
            D_beta=abs(C2*Beta_pos(j)-Positions(i,j)); % Equation (3.5)-part 2
            X2=Beta_pos(j)-A2*D_beta; % Equation (3.6)-part 2
            
            r1=rand();
            r2=rand();
            
            A3=2*a*r1-a; % Equation (3.3)
            C3=2*r2; % Equation (3.4)
            
            D_delta=abs(C3*Delta_pos(j)-Positions(i,j)); % Equation (3.5)-part 3
            X3=Delta_pos(j)-A3*D_delta; % Equation (3.5)-part 3
            
            Positions(i,j)=(X1+X2+X3)/3;% Equation (3.7)
            
        end
    end
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

function [Positions]=corOppose(Positions,fitness,ub,lb,upper,lower,dim,threshold)




%fprintf('**%d %d %d %d\n',size(upper,1),size(upper,2),size(lower,1),size(lower,2))

n=size(fitness);
for i=4:n(1)
    sum=0;
    greater=[];
    less=[];
    x=1;z=1;y=1;
    
    for j=1:size(Positions(1,:),2)
        
        d(x)=abs(Positions(1,j)-Positions(i,j));
        if d(x)<threshold
            greater(y)=j;
            y=y+1;
        else
            less(z)=j;
            z=z+1;
        end
        sum=sum+d(x)*d(x);
        x=x+1;
    end
%     sum
    src=1-(double(6*sum))/(double(n(1)*(n(1)*n(1)-1)));
%     src
    if src<=0
        if size(greater)<size(less)
%             for j=1:size(less)
%                 dim=less(j);
%                 Positions(i,dim)=ub(dim)+lb(dim)-Positions(i,dim);
%             end
        else
            for j=1:size(greater)
                dim=greater(j);
                Positions(i,dim)=upper(1,dim)+lower(1,dim)-Positions(i,dim);
            end
        end
    end
end
end





