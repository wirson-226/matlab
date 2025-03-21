% VAGWO algorithm                                                                  
function [z_final,z_iter,z_optimal] = VAGWO(np,maxit,varmin,varmax,nx,fobj)
it=1;
elitism=1; % elitism=1: Elitism is applied; elitism=any other number: Elitism is not applied 
a_max=sqrt(2); % Upper bound of the acceleration coefficient
a_min=0; % Lower bound of the acceleration coefficient
c_max=1; % Upper bound of the leading wolves multipliers
c_min=0; % Lower bound of the leading wolves multipliers
k_max=0.9; % Upper bound of the inertia weight
k_min=0.4; % Lower bound of the inertia weight
limvel=0.1; % A ratio of the maximum distance in the search space to form the maximum velocity
velmax=limvel*(varmax(1,1:nx)-varmin(1,1:nx)); % Upper bound defined for the velocities
velmin=-velmax; % Lower bound defined for the velocities

% disp(['Number of Iterations = ',num2str(it)]);
pp_pbest=zeros(np,nx);
pv=zeros(np,nx);
optimal_pos=zeros(1,nx);
z=zeros(np);
z_pbest=zeros(np);
pos_final=zeros(nx);
z_iter=zeros(maxit);
z_alpha=inf;
z_beta=inf;
z_delta=inf;

% Initialization process of the algorithm
[pp,gv_alpha,gv_beta,gv_delta]=Initialization(np,nx,varmax,varmin,velmax,velmin);

% Start the optimization process

% Objective function evaluations and determine the personal best solutions
% and objectives, if elitism is applied
for j=1:np
    z(j)=fobj(pp(j,1:nx));
    
    % Elitism
    if elitism==1
        z_pbest(j)=z(j);
        pp_pbest(j,1:nx)=pp(j,1:nx);
    end
end
for j=1:np
    if z(j)<=z_alpha
        z_alpha=z(j);
        alpha(1,1:nx)=pp(j,1:nx);
    elseif z(j)>z_alpha && z(j)<=z_beta
        z_beta=z(j);
        beta(1,1:nx)=pp(j,1:nx);
    elseif z(j)>z_beta && z(j)<=z_delta
        z_delta=z(j);
        delta(1,1:nx)=pp(j,1:nx);
    end
end
z_optimal(it)=z_alpha;
optimal_pos(it,:)=alpha(1,:);

% Save the best-so-far objective value in the current run
z_iter(it)=z_optimal(it);

% The Main Loop
while it<maxit
    it=it + 1;
    aa=a_max-(a_max-a_min)*(it-1)/(maxit-1); % Eq.(16)
    cc=c_max-(c_max-c_min)*(it-1)/(maxit-1); % Eq.(23)
    k=k_max-(k_max-k_min)*(it-1)/(maxit-1); % Eq.(24)
%     disp(['Number of Iterations= ',num2str(it)]);
    for j=1:np        
        a_alpha(1,1:nx)=(2*rand(1,nx)-ones(1,nx))*(aa^2); % Eq.(13)
        c_alpha(1,1:nx)=ones(1,nx)+(2*rand(1,nx)-ones(1,nx))*(cc^2); % Eq.(20)
        gv_alpha(j,1:nx)=k*(sign(a_alpha(1,1:nx)).*abs(gv_alpha(j,1:nx)))+...
            a_alpha(1,1:nx).*abs(c_alpha(1,1:nx).*alpha(1,1:nx)-pp(j,1:nx)); % Eq.(10)
        
        a_beta(1,1:nx)=(2*rand(1,nx)-ones(1,nx))*(aa^2); % Eq.(14)
        c_beta(1,1:nx)=ones(1,nx)+(2*rand(1,nx)-ones(1,nx))*(cc^2); % Eq.(21)
        gv_beta(j,1:nx)=k*(sign(a_beta(1,1:nx)).*abs(gv_beta(j,1:nx)))+...
            a_beta(1,1:nx).*abs(c_beta(1,1:nx).*beta(1,1:nx)-pp(j,1:nx)); % Eq.(11)
        
        a_delta(1,1:nx)=(2*rand(1,nx)-ones(1,nx))*(aa^2); % Eq.(15)
        c_delta(1,1:nx)=ones(1,nx)+(2*rand(1,nx)-ones(1,nx))*(cc^2); % Eq.(22);
        gv_delta(j,1:nx)=k*(sign(a_delta(1,1:nx)).*abs(gv_delta(j,1:nx)))+...
            a_delta(1,1:nx).*abs(c_delta(1,1:nx).*delta(1,1:nx)-pp(j,1:nx)); % Eq.(12)
        
        sum1=gv_alpha(j,:)+gv_beta(j,:)+gv_delta(j,:);
        sum2=alpha+beta+delta;
        pv(j,1:nx)=sum1/3;
        
        % Return back the velocity of the particles if going beyond the velocity boundaries
        flag4lbv=pv(j,:)<velmin(1,:);
        flag4ubv=pv(j,:)>velmax(1,:);
        pv(j,:)=(pv(j,:)).*(~(flag4lbv+flag4ubv))+velmin.*flag4lbv+velmax.*flag4ubv;
        
        pp(j,1:nx)=sum2/3-pv(j,:); % Eq.(28)
            
        % Return back the position of the particles if going beyond the position boundaries
        flag4lbp=pp(j,:)<varmin(1,:);
        flag4ubp=pp(j,:)>varmax(1,:);
        pp(j,:)=(pp(j,:)).*(~(flag4lbp+flag4ubp))+varmin.*flag4lbp+varmax.*flag4ubp; 
    
    % Objective function evaluations and determining of the personal best solutions and objectives
        z(j)=fobj(pp(j,:));
    end
    if elitism==1
        for j=1:np
            if z_pbest(j)<z(j)
                z(j)=z_pbest(j);
                pp(j,:)=pp_pbest(j,:);
            else
                z_pbest(j)=z(j);
                pp_pbest(j,:)=pp(j,:);
            end
        end
    end
    for j=1:np
        if z(j)<=z_alpha
            z_alpha=z(j);
            alpha(1,1:nx)=pp(j,1:nx);
        elseif z(j)>z_alpha && z(j)<=z_beta
            z_beta=z(j);
            beta(1,1:nx)=pp(j,1:nx);
        elseif z(j)>z_beta && z(j)<=z_delta
            z_delta=z(j);
            delta(1,1:nx)=pp(j,1:nx);
        end
    end
    z_optimal(it)=z_alpha;
    optimal_pos(it,:)=alpha(1,:);
    
    % Save the best-so-far objective value in the current run
    z_iter(it)=z_optimal(it);
end

% Save the final best solution and objective revealed upon the end of the optimization process
z_final=z_optimal(maxit);
pos_final(1:nx)=optimal_pos(maxit,1);
% pos_final = optimal_pos(1:maxit, 1);


end

% This function is to initialize the position and velocity of the wolves to start the optimization process
function [pp,gv_alpha,gv_beta,gv_delta] = Initialization(np,nx,varmax,varmin,velmax,velmin)
pp=zeros(np,nx); 
gv_alpha=zeros(np,nx);
gv_beta=zeros(np,nx);
gv_delta=zeros(np,nx);
for j=1:np
    pp(j,1:nx)=(varmax-varmin).*rand(1,nx)+varmin;
    gv_alpha(j,1:nx)=(velmax-velmin).*rand(1,nx)+velmin;
    gv_beta(j,1:nx)=(velmax-velmin).*rand(1,nx)+velmin;
    gv_delta(j,1:nx)=(velmax-velmin).*rand(1,nx)+velmin;
end
end