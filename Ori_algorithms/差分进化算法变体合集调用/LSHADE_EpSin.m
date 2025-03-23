%%%%%%%%%%%%%%%%%%%
%% This package is a MATLAB/Octave source code of LSHADE_EpSin which is an improved version of L-SHADE.
%% Please see the following paper:
%% * Noor H. Awad, Mostafa Z. Ali, Ponnuthurai N. Suganthan and Robert G. Reynolds: An Ensemble Sinusoidal Parameter Adaptation incorporated with L-SHADE for Solving CEC2014 Benchmark Problems, Proc. IEEE Congress on Evolutionary Computation (CEC-2016), Canada, July, 2016
%% please follow Wechat Swarm-Opti
function [bsf_fit_var,bsf_solution,cg]=LSHADE_EpSin(pop_size,max_nfes,lb,ub,dim,fobj)

%%% change freq
freq_inti = 0.5;
lu = [lb.* ones(1, dim); ub.* ones(1, dim)];

pb = 0.4;
ps = .5;

S.Ndim = dim;
S.Lband = ones(1, S.Ndim)*(-100);
S.Uband = ones(1, S.Ndim)*(100);

GenMaxSelected = 250; %%% For local search

%%%% Count the number of maximum generations before as NP is dynamically
%%%% decreased
G_Max = 0;
if dim == 10
    G_Max = 2163;
end
if dim == 30
    G_Max = 2745;
end
if dim == 50
    G_Max = 3022;
end
if dim == 100
    G_Max = 3401;
end


%%  parameter settings for L-SHADE
p_best_rate = 0.11;    %0.11
arc_rate = 1.4;
memory_size = 5;
SEL = round(ps*pop_size);

max_pop_size = pop_size;
min_pop_size = 4.0;
nfes = 0;
%% Initialize the main population
popold = repmat(lu(1, :), pop_size, 1) + rand(pop_size, dim) .* (repmat(lu(2, :) - lu(1, :), pop_size, 1));
pop = popold; % the old population becomes the current population

for it=1:pop_size
    fitness(it) = fobj( pop(it,:));
end
fitness = fitness';

%%% Initialize LS population
flag_LS = false;
counter = 0;
popsize_LS = 10;

%%% Initialize LS population for re-start
popLS = repmat(lu(1, :), popsize_LS, 1) + rand(popsize_LS, dim) .* (repmat(lu(2, :) - lu(1, :), popsize_LS, 1));
for it=1:popsize_LS
    fitness_LS(it) = fobj( pop(it,:));
end
fitness_LS = fitness_LS';

%%%%%%%%%%%%%

[Sorted_FitVector, Indecis] = sort(fitness_LS);
popLS = popLS(Indecis,:);%sorting the points based on obtaind result
%==========================================================================

%Finding the Best point in the group=======================================
BestPoint = popLS(1, :);
F = Sorted_FitVector(1);%saving the first best fitness
%%%%%%%%%%%%%

bsf_fit_var = 1e+30;
bsf_index = 0;
bsf_solution = zeros(1, dim);

%%%%%%%%%%%%%%%%%%%%%%%% for out
for i = 1 : pop_size
    
    
    if fitness(i) < bsf_fit_var
        bsf_fit_var = fitness(i);
        bsf_solution = pop(i, :);
        bsf_index = i;
    end
    
    if nfes > max_nfes; break; end
end
%%%%%%%%%%%%%%%%%%%%%%%% for out

memory_sf = 0.5 .* ones(memory_size, 1);
memory_cr = 0.5 .* ones(memory_size, 1);

memory_freq = freq_inti*ones(memory_size, 1);
memory_pos = 1;

archive.NP = arc_rate * pop_size; % the maximum size of the archive
archive.pop = zeros(0, dim); % the solutions stored in te archive
archive.funvalues = zeros(0, 1); % the function value of the archived solutions

%% main loop
gg=0;  %%% generation counter used For Sin
igen =1;  %%% generation counter used For LS

flag1 = false;
flag2 = false;
while nfes < max_nfes
    gg=gg+1;
    
    pop = popold; % the old population becomes the current population
    [temp_fit, sorted_index] = sort(fitness, 'ascend');
    
    mem_rand_index = ceil(memory_size * rand(pop_size, 1));
    mu_sf = memory_sf(mem_rand_index);
    mu_cr = memory_cr(mem_rand_index);
    mu_freq = memory_freq(mem_rand_index);
    
    %% for generating crossover rate
    cr = normrnd(mu_cr, 0.1);
    term_pos = find(mu_cr == -1);
    cr(term_pos) = 0;
    cr = min(cr, 1);
    cr = max(cr, 0);
    
    %% for generating scaling factor
    sf = mu_sf + 0.1 * tan(pi * (rand(pop_size, 1) - 0.5));
    pos = find(sf <= 0);
    
    while ~ isempty(pos)
        sf(pos) = mu_sf(pos) + 0.1 * tan(pi * (rand(length(pos), 1) - 0.5));
        pos = find(sf <= 0);
    end
    
    
    freq = mu_freq + 0.1 * tan(pi*(rand(pop_size, 1) - 0.5));
    pos_f = find(freq <=0);
    while ~ isempty(pos_f)
        freq(pos_f) = mu_freq(pos_f) + 0.1 * tan(pi * (rand(length(pos_f), 1) - 0.5));
        pos_f = find(freq <= 0);
    end
    
    sf = min(sf, 1);
    freq = min(freq, 1);
    
    if(nfes <= max_nfes/2)
        c=rand;
        if(c<0.5)
            sf = 0.5.*( sin(2.*pi.*freq_inti.*gg+pi) .* ((G_Max-gg)/G_Max) + 1 ) .* ones(pop_size,dim);
        else
            sf = 0.5 *( sin(2*pi .* freq(:, ones(1, dim)) .* gg) .* (gg/G_Max) + 1 ) .* ones(pop_size,dim);
        end
    end
    
    r0 = [1 : pop_size];
    popAll = [pop; archive.pop];
    [r1, r2] = gnR1R2(pop_size, size(popAll, 1), r0);
    
    pNP = max(round(p_best_rate * pop_size), 2); %% choose at least two best solutions
    randindex = ceil(rand(1, pop_size) .* pNP); %% select from [1, 2, 3, ..., pNP]
    randindex = max(1, randindex); %% to avoid the problem that rand = 0 and thus ceil(rand) = 0
    pbest = pop(sorted_index(randindex), :); %% randomly choose one of the top 100p% solutions
    
    vi = pop + sf(:, ones(1, dim)) .* (pbest - pop + pop(r1, :) - popAll(r2, :));
    vi = boundConstraint(vi, pop, lu);
    
    mask = rand(pop_size, dim) > cr(:, ones(1, dim)); % mask is used to indicate which elements of ui comes from the parent
    rows = (1 : pop_size)'; cols = floor(rand(pop_size, 1) * dim)+1; % choose one position where the element of ui doesn't come from the parent
    jrand = sub2ind([pop_size dim], rows, cols); mask(jrand) = false;
    ui = vi; ui(mask) = pop(mask);
    children_fitness=[];
    for i=1:size(ui,1)
        children_fitness(i,:) = fobj(ui(i,:));
    end

    %%%% To check stagnation
    flag = false;
    bsf_fit_var_old = bsf_fit_var;
    %%%%%%%%%%%%%%%%%%%%%%%% for out
    for i = 1 : pop_size
        
        if children_fitness(i) < bsf_fit_var
            bsf_fit_var = children_fitness(i);
            bsf_solution = ui(i, :);
            bsf_index = i;
        end
        
        if nfes > max_nfes; break; end
    end
    %%%%%%%%%%%%%%%%%%%%%%%% for out
    
    dif = abs(fitness - children_fitness);
    
    
    %% I == 1: the parent is better; I == 2: the offspring is better
    I = (fitness > children_fitness);
    goodCR = cr(I == 1);
    goodF = sf(I == 1);
    goodFreq = freq(I == 1);
    dif_val = dif(I == 1);

    archive = updateArchive(archive, popold(I == 1, :), fitness(I == 1));
    
    [fitness, I] = min([fitness, children_fitness], [], 2);
    
    
    popold = pop;
    popold(I == 2, :) = ui(I == 2, :);
    
    num_success_params = numel(goodCR);
    
    if num_success_params > 0
        sum_dif = sum(dif_val);
        dif_val = dif_val / sum_dif;
        
        %% for updating the memory of scaling factor
        memory_sf(memory_pos) = (dif_val' * (goodF .^ 2)) / (dif_val' * goodF);
        
        %% for updating the memory of crossover rate
        if max(goodCR) == 0 || memory_cr(memory_pos)  == -1
            memory_cr(memory_pos)  = -1;
        else
            memory_cr(memory_pos) = (dif_val' * (goodCR .^ 2)) / (dif_val' * goodCR);
        end
        
        %% for updating the memory of freq
        if max(goodFreq) == 0 || memory_freq(memory_pos)  == -1
            memory_freq(memory_pos)  = -1;
        else
            memory_freq(memory_pos) = (dif_val' * (goodFreq .^ 2)) / (dif_val' * goodFreq);
        end
        
        memory_pos = memory_pos + 1;
        if memory_pos > memory_size;  memory_pos = 1; end
    end
    
    %% for resizing the population size
    plan_pop_size = round((((min_pop_size - max_pop_size) / max_nfes) * nfes) + max_pop_size);
    
    if pop_size > plan_pop_size
        reduction_ind_num = pop_size - plan_pop_size;
        if pop_size - reduction_ind_num <  min_pop_size; reduction_ind_num = pop_size - min_pop_size;end
        
        pop_size = pop_size - reduction_ind_num;
        SEL = round(ps*pop_size);
        for r = 1 : reduction_ind_num
            [valBest indBest] = sort(fitness, 'ascend');
            worst_ind = indBest(end);
            popold(worst_ind,:) = [];
            pop(worst_ind,:) = [];
            fitness(worst_ind,:) = [];
        end
        
        archive.NP = round(arc_rate * pop_size);
        
        if size(archive.pop, 1) > archive.NP
            rndpos = randperm(size(archive.pop, 1));
            rndpos = rndpos(1 : archive.NP);
            archive.pop = archive.pop(rndpos, :);
        end
    end
    
    %%%%%%%%%%%%%%% Call LS based on Gaussian works when NP is less than 20 for the first time  %%%%%
    if pop_size <= 20
        counter = counter + 1;
    end
    
    if counter == 1
        flag_LS = true;
    else
        flag_LS = false;
    end
    
    if flag_LS == true
        r_index = randi([1 pop_size],1,popsize_LS);
        %%% Pick 10 random individuals from L-SHADE pop
        for gen_LS = 0 : GenMaxSelected
            New_Point = [];%creating new point
            FitVector = [];%creating vector of fitness functions
            
            for i = 1 : popsize_LS
                [NP, fit] = LS_Process(popLS(i,:),S,gg,BestPoint,fobj);
                New_Point = [New_Point;NP];
                FitVector = [FitVector,fit];
            end
            
            %%%%
            fittemp = FitVector;
            for i = 1 : popsize_LS
                %%% Update those 10 random individuals from pop L-SHADE
                if FitVector(i) < fitness(r_index(i))
                    fitness (r_index(i)) = FitVector(i);
                    pop(r_index(i),:) = New_Point(i,:);
                    
                else
                    fittemp(i) =  fitness (r_index(i));
                end
                
                %%%% Update best individual L-SHADE
                if FitVector(i) < bsf_fit_var
                    bsf_fit_var = FitVector(i);
                    bsf_solution = New_Point(i,:);
                end
                
                if nfes > max_nfes; break; end
            end
            
            %%%%%% To recored those changes
            fittemp = fittemp';
            
            %%%%%%%%%%%%%%
            
            [SortedFit,SortedIndex] = sort(FitVector);
            New_Point = New_Point(SortedIndex,:);
            BestPoint = New_Point(1,:);%first point is the best
            BestFitness = SortedFit(1,1);
            popLS = New_Point;
        end
    end
    nfes = nfes + 1;
    cg(nfes) = bsf_fit_var;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end %%%%%%%%nfes

end


function [r1, r2] = gnR1R2(NP1, NP2, r0)

% gnA1A2 generate two column vectors r1 and r2 of size NP1 & NP2, respectively
%    r1's elements are choosen from {1, 2, ..., NP1} & r1(i) ~= r0(i)
%    r2's elements are choosen from {1, 2, ..., NP2} & r2(i) ~= r1(i) & r2(i) ~= r0(i)
%
% Call:
%    [r1 r2 ...] = gnA1A2(NP1)   % r0 is set to be (1:NP1)'
%    [r1 r2 ...] = gnA1A2(NP1, r0) % r0 should be of length NP1
%
% Version: 2.1  Date: 2008/07/01
% Written by Jingqiao Zhang (jingqiao@gmail.com)

NP0 = length(r0);

r1 = floor(rand(1, NP0) * NP1) + 1;
%for i = 1 : inf
for i = 1 : 99999999
    pos = (r1 == r0);
    if sum(pos) == 0
        break;
    else % regenerate r1 if it is equal to r0
        r1(pos) = floor(rand(1, sum(pos)) * NP1) + 1;
    end
    if i > 1000, % this has never happened so far
        error('Can not genrate r1 in 1000 iterations');
    end
end

r2 = floor(rand(1, NP0) * NP2) + 1;
%for i = 1 : inf
for i = 1 : 99999999
    pos = ((r2 == r1) | (r2 == r0));
    if sum(pos)==0
        break;
    else % regenerate r2 if it is equal to r0 or r1
        r2(pos) = floor(rand(1, sum(pos)) * NP2) + 1;
    end
    if i > 1000, % this has never happened so far
        error('Can not genrate r2 in 1000 iterations');
    end
end
end


%This function is used as a local search, and creates some
%new points based on Gaussian Walks.

%**************************************************************************
%The input function is:                                                   %
%Point: the input point which is going to be diffused                     %
%S: structure of problem information                                      %
%g: generation number                                                     %
%BestPoint: the best point in group                                       %                               %
%==========================================================================
%The output function is:                                                  %
%createPoint: the new points created by Diffusion process                 %
%fitness: the value of fitness function                                   %
%**************************************************************************

function [createPoint, fitness] = LS_Process(Point,S,g,BestPoint,fobj)


GeneratePoint = normrnd(BestPoint, (log(g)/g)*(abs((Point - BestPoint))), [1 size(Point,2)]) + ...
    (randn*BestPoint - randn*Point);

%check bounds of generated point
GeneratePoint = Bound_Checking(GeneratePoint,S.Lband,S.Uband);

%     size(GeneratePoint)
%     for i=1:size(Point,2)
%         if GeneratePoint(1,i) > S.Uband
%             fprintf('violate upper');
%         end
%         if GeneratePoint(1,i) < S.Lband
%              fprintf('violate lower');
%         end
%     end

fitness = fobj(GeneratePoint);

createPoint = GeneratePoint;
%======================================================================
end
function archive = updateArchive(archive, pop, funvalue)
% Update the archive with input solutions
%   Step 1: Add new solution to the archive
%   Step 2: Remove duplicate elements
%   Step 3: If necessary, randomly remove some solutions to maintain the archive size
%
% Version: 1.1   Date: 2008/04/02
% Written by Jingqiao Zhang (jingqiao@gmail.com)

if archive.NP == 0, return; end

if size(pop, 1) ~= size(funvalue,1), error('check it'); end

% Method 2: Remove duplicate elements
popAll = [archive.pop; pop ];
funvalues = [archive.funvalues; funvalue ];
[dummy IX]= unique(popAll, 'rows');
if length(IX) < size(popAll, 1) % There exist some duplicate solutions
    popAll = popAll(IX, :);
    funvalues = funvalues(IX, :);
end

if size(popAll, 1) <= archive.NP   % add all new individuals
    archive.pop = popAll;
    archive.funvalues = funvalues;
else                % randomly remove some solutions
    rndpos = randperm(size(popAll, 1)); % equivelent to "randperm";
    rndpos = rndpos(1 : archive.NP);
    
    archive.pop = popAll  (rndpos, :);
    archive.funvalues = funvalues(rndpos, :);
end
end
%This function is used for LS bound checking
function p = Bound_Checking(p,lowB,upB)
for i = 1 : size(p,1)
    upper = double(gt(p(i,:),upB));
    lower = double(lt(p(i,:),lowB));
    up = find(upper == 1);
    lo = find(lower == 1);
    if (size(up,2)+ size(lo,2) > 0 )
        for j = 1 : size(up,2)
            %                 fprintf('here');
            p(i, up(j)) = (upB(up(j)) - lowB(up(j)))*rand()...
                + lowB(up(j));
        end
        for j = 1 : size(lo,2)
            p(i, lo(j)) = (upB(lo(j)) - lowB(lo(j)))*rand()...
                + lowB(lo(j));
        end
    end
end
end


%This function is used for L-SHADE bound checking
function vi = boundConstraint (vi, pop, lu)

% if the boundary constraint is violated, set the value to be the middle
% of the previous value and the bound
%

[NP, D] = size(pop);  % the population size and the problem's dimension

%% check the lower bound
xl = repmat(lu(1, :), NP, 1);

pos = vi < xl;
vi(pos) = (pop(pos) + xl(pos)) / 2;

%% check the upper bound
xu = repmat(lu(2, :), NP, 1);
pos = vi > xu;
vi(pos) = (pop(pos) + xu(pos)) / 2;
end