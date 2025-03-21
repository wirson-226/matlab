function [BestSol9,BestPos9,BestCost9]=MELGWO(SearchAgents_no,Max_iter,lb,ub,dim,fobj)
%% Problem Definition

% BASIC / GENERAL PARAMETERS 
% fobj= @(x)Sphere(x);  % Cost Function
nVar = dim;            % Maximum number of Decision Variables or Dimensions of the problem 
m = [1 nVar];         % Size of Decision Variables Matrix
            % Upper bound 
Agents_no = SearchAgents_no;      % Number of search agents

FE =0;                % FUNCTION EVALUATION COUNTER 
Max_FE = 50000;       % MAXIMUM NUMBER OF FUNCTION EVALUATION
l=0;% Loop counter

Convergence_curve=zeros(1,Max_iter); % Array for storing best function values to plot the convergence curve

% ALGORITHM SPECIFIC PARAMETERS 
Min_Agents_no = 10;
Max_Agents_no = Agents_no;
Fmin = 0.10;
Fmax = 2.00;
pCR = 0.60;
bounds(:,1) = lb;
bounds(:,2) = ub;
wmax = 1;
wmin = 0.1;


VRmin = bounds(:,1)';
VRmax = bounds(:,2)';

if length(VRmin)==1
    VRmin=repmat(VRmin,1,nVar);
    VRmax=repmat(VRmax,1,nVar);
end
mv=0.5*(VRmax-VRmin);
VRmin=repmat(VRmin,Agents_no,1);
VRmax=repmat(VRmax,Agents_no,1);
Vmin=repmat(-mv,Agents_no,1);
Vmax=-Vmin;

% initialize alpha, beta, and delta_pos
Alpha_pos=zeros(1,nVar);
Alpha_score=inf; %change this to -inf for maximization problems

Beta_pos=zeros(1,nVar);
Beta_score=inf; %change this to -inf for maximization problems

Delta_pos=zeros(1,nVar);
Delta_score=inf; %change this to -inf for maximization problems

%% INITIALIZATION OF WOLVES 
Boundary_no = size(ub,2); % numnber of boundaries
% If the boundaries of all variables are equal, then user needs to enter two different signle
% numbers for both ub and lb (like lb = -100; ub = 100;)

% if Boundary_no==1
for i = 1:Agents_no
    Agents(i).Position=unifrnd(lb,ub,m);
    Agents(i).fitness=fobj(Agents(i).Position);
    FE=FE+1;
end

for i = 1:Agents_no
    Agents_Position_Memory_old(i,:) = Agents(i).Position;
    Agents_Cost_Memory_old(i) = Agents(i).fitness;
end

Agents_Position_Memory = Agents_Position_Memory_old;
Agents_Cost_Memory = Agents_Cost_Memory_old;

%ENDING OF INITIALIZATION
  
%% Selecting alpha, beta, and delta from the initials 

for i = 1: Agents_no

        if Agents(i).fitness < Alpha_score
            Alpha_score = Agents(i).fitness; % Update alpha
            Alpha_pos = Agents(i).Position;
        end

        if Agents(i).fitness > Alpha_score && Agents(i).fitness < Beta_score
            Beta_score = Agents(i).fitness; % Update beta
            Beta_pos = Agents(i).Position;
        end

        if Agents(i).fitness > Alpha_score && Agents(i).fitness > Beta_score && Agents(i).fitness < Delta_score
            Delta_score = Agents(i).fitness; % Update delta
            Delta_pos = Agents(i).Position;
        end

end


%  Main Loop 
%% GREY WOLF OPTIMIZER PART 

while l < Max_iter
    for i=1:Agents_no
        
        % Checking boundaries 
        Agents(i).Position = max(Agents(i).Position,lb);
        Agents(i).Position = min(Agents(i).Position,ub);
        Agents_Position_Memory(i,:) = max(Agents_Position_Memory(i,:),lb);
        Agents_Position_Memory(i,:) = min(Agents_Position_Memory(i,:),ub);
        
        % Update Alpha, Beta, and Delta from the Explorer swarm 
        if Agents(i).fitness < Alpha_score
            Alpha_score = Agents(i).fitness; % Update alpha
            Alpha_pos = Agents(i).Position;
        end
        
        if Agents(i).fitness > Alpha_score && Agents(i).fitness < Beta_score
            Beta_score = Agents(i).fitness; % Update beta
            Beta_pos = Agents(i).Position;
        end
        
        if Agents(i).fitness > Alpha_score && Agents(i).fitness > Beta_score && Agents(i).fitness < Delta_score
            Delta_score = Agents(i).fitness; % Update delta
            Delta_pos = Agents(i).Position;
        end
    end
    
    a=2-l*((2)/Max_iter); % a decreases linearly fron 2 to 0
    
    % Update the Position of search agents including omegas
    for i=1:Agents_no
        
        r1=rand(); % r1 is a random number in [0,1]
        r2=rand(); % r2 is a random number in [0,1]
        
        A1=2*a*r1-a; % Equation (3.3)
        C1=2*r2; % Equation (3.4)
        
        D_alpha=abs(C1*Alpha_pos-Agents(i).Position); % Equation (3.5)-part 1
        X1=Alpha_pos -A1*D_alpha; % Equation (3.6)-part 1
        
        r1=rand();
        r2=rand();
        
        A2=2*a*r1-a; % Equation (3.3)
        C2=2*r2; % Equation (3.4)
        
        D_beta=abs(C2*Beta_pos-Agents(i).Position); % Equation (3.5)-part 2
        X2=Beta_pos-A2*D_beta; % Equation (3.6)-part 2
        
        r1=rand();
        r2=rand();
        
        A3=2*a*r1-a; % Equation (3.3)
        C3=2*r2; % Equation (3.4)
        
        D_delta=abs(C3*Delta_pos-Agents(i).Position); % Equation (3.5)-part 3
        X3=Delta_pos-A3*D_delta; % Equation (3.5)-part 3
        
        Agents(i).Position=(X1+X2+X3)/3;% Equation (3.7)
        
        % Calculate objective function for each search agent
        Agents(i).Position = max(Agents(i).Position,lb);
        Agents(i).Position = min(Agents(i).Position,ub);
        Agents(i).fitness=fobj(Agents(i).Position);
        % FE=FE+1;
        
        
    end
 %% EVOLUTIONARY OPERATORS FROM DE 

    for i=1:Agents_no
        x = Agents(i).Position;
        F =  Fmin + (Fmax - Fmin) * ((Max_iter-(l-1))/Max_iter);
        %y =   Alpha_pos + F*(Beta_pos - Delta_pos);
        y =  x  + F.*(Alpha_pos - x);
        
        % Crossover
        z=zeros(size(x));
        j0=randi([1 numel(x)]);
        for j=1:numel(x)
            if j==j0 || rand<=pCR
                z(j)=y(j);
            else
                z(j)=x(j);
            end
        end
        
        NewSol.Position=z;
        
        NewSol.Position = max(NewSol.Position,lb);
        NewSol.Position = min(NewSol.Position,ub);
        
        NewSol.Cost= fobj(NewSol.Position);
        % FE = FE+1;
        
        if NewSol.Cost < Agents(i).fitness
            Agents(i).Position =  NewSol.Position;
            Agents(i).fitness =  NewSol.Cost;
        end
    end
    
    %% UPDATING THE MEMORY 
    for i = 1:Agents_no
        if Agents(i).fitness < Agents_Cost_Memory(i)
            Agents_Position_Memory(i,:) = Agents(i).Position;
            Agents_Cost_Memory(i) = Agents(i).fitness;
        end
    end
    
    %% SORTING MEMORY
    [Agents_Cost_Memory]=[Agents_Cost_Memory];
    [Agents_Cost_Memory, SortOrder]=sort(Agents_Cost_Memory);
    Agents_Position_Memory = Agents_Position_Memory(SortOrder,:);
    
    %% STARTING OF LOCAL SEARCH PART
    
    for k=1: Agents_no/2
        [temp kk] =sort (sqrt(sum((ones(Agents_no,1)*Agents_Position_Memory(k,:) - Agents_Position_Memory).^2,2)));%distance
        if Agents_Cost_Memory(kk(1)) <= Agents_Cost_Memory(kk(2))
            nbest1 = Agents_Position_Memory(kk(1),:);
            nbest2 = Agents_Position_Memory(kk(2),:);% particle_Position_Memory , particle_fitness_Memory
        else
            nbest2=Agents_Position_Memory(kk(1),:);
            nbest1=Agents_Position_Memory(kk(2),:);
        end
        
        temp_pbest=Agents_Position_Memory(k,:) + 2*rand(1,nVar).*(nbest1-nbest2);%nearest n best
        % temp_pbest= particle_Position_Memory(k,:)+2*randn(1,D).*(nbest1-nbest2);
        temp_pbest = ((temp_pbest >= VRmin(1,:))&(temp_pbest <= VRmax(1,:))).*temp_pbest...
            +(temp_pbest < VRmin(1,:)).*(VRmin(1,:) + 0.25.*(VRmax(1,:) - VRmin(1,:)).*rand(1,nVar)) ...
            + (temp_pbest > VRmax(1,:)).*(VRmax(1,:) - 0.25.*(VRmax(1,:) - VRmin(1,:)).*rand(1,nVar));
        
        
        temp_pbest_val = fobj (temp_pbest);%benchmark_functions(temp_pbest,Benchmark_Function_ID,D);
        % FE=FE+1;
        
        tmp = (Agents_Cost_Memory(k) < temp_pbest_val);
        temp = repmat(tmp,1,nVar);
        Agents_Position_Memory(k,:) = temp.*Agents_Position_Memory(k,:) + (1-temp).*temp_pbest;
        Agents_Cost_Memory(k) = tmp.*Agents_Cost_Memory(k) + (1-tmp).*temp_pbest_val;%update the pbest LOCAL SESCRCJ particle_Position_Memory(j,:);
        %                                       nbestval(i)=particle_fitness_Memory(j);
        
        
        Agents_Position_Memory(k,:) = max(Agents_Position_Memory(k,:),lb);
        Agents_Position_Memory(k,:) = min(Agents_Position_Memory(k,:),ub);
        
 %% UPDATINH ALPHA, BETA, AND DELTA FROM THE MEMORY SWARM ALSO
        
        if Agents_Cost_Memory(k) < Alpha_score
            
            Alpha_pos =   Agents_Position_Memory(k,:);
            Alpha_score = Agents_Cost_Memory(k);
        end
        
        if Agents_Cost_Memory(k) < Beta_score && Agents_Cost_Memory(k) > Alpha_score
            
            Beta_pos =   Agents_Position_Memory(k,:);
            Beta_score = Agents_Cost_Memory(k);
        end
        
        if Agents_Cost_Memory(k) < Delta_score && Agents_Cost_Memory(k) > Alpha_score && Agents_Cost_Memory(k) > Beta_score
            
            Delta_pos =   Agents_Position_Memory(k,:);
            Delta_score = Agents_Cost_Memory(k);
        end
    end
    
    
 %% LINEAR POPULATION SIZE REDUCTION  
    
    Plan_Agent_no = round((((Min_Agents_no - Max_Agents_no) / Max_FE) * FE) + Max_Agents_no);
    
    if Agents_no > Plan_Agent_no
        reduction_ind_num = Agents_no - Plan_Agent_no;
        if Agents_no - reduction_ind_num <  Min_Agents_no; reduction_ind_num = Agents_no - Min_Agents_no;
        end
        
        Agents_no = Agents_no - reduction_ind_num;
        
        for r = 1 : reduction_ind_num
            [fitness]=[Agents.fitness];
            [fitness, SortOrder]=sort(fitness);
            Agents = Agents(SortOrder);
            worst_ind = SortOrder(end);
            Agents = Agents(1:Agents_no);
            fitness = fitness(1:Agents_no);
            Agents_Position_Memory(worst_ind,:) = [];
            Agents_Cost_Memory = Agents_Cost_Memory(1:Agents_no);
        end
    end
    
 %% STORING / SAVING RESULTS 
    l=l+1;
    Convergence_curve(l)=Alpha_score;
    BestSol9 = Alpha_score ;
    BestPos9 = Alpha_pos ;
    BestCost9 = Convergence_curve ;
    % disp(['Iteration ' num2str(l) ': Best Cost = ' num2str( Convergence_curve(l))]);
    
end

% %% Drawing / plotting objective space
% semilogy(Convergence_curve,'-x', 'LineWidth',2)
% title('Objective space')
% xlabel('Iteration');
% ylabel('Best score obtained so far');
% grid on;