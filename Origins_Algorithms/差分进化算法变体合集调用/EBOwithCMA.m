function [bestfit, bestpos, Cg_curve] = EBOwithCMA(N, T, lb, ub, dim, fobj)
    % EBO_BIN: An implementation of the EBO algorithm for optimization.
    %
    % Inputs:
    %   - N: Population size
    %   - T: Maximum number of iterations
    %   - lb: Lower bounds (1 x dim)
    %   - ub: Upper bounds (1 x dim)
    %   - dim: Dimension of the problem
    %   - fobj: Objective function handle
    %
    % Outputs:
    %   - bestfit: Best fitness value found
    %   - bestpos: Best solution found
    %   - Cg_curve: Convergence curve (best fitness value at each iteration)

    %% Initialize parameters
    current_eval = 0;  % Current number of function evaluations
    iter = 0;          % Current iteration
    n = dim;           % Dimension
    Max_FES = N * T;   % Maximum number of function evaluations

    % Initialize population sizes
    PopSize1 = N;                      % Size of first subpopulation
    PopSize2 = 4 + floor(3 * log(n));  % Size of second subpopulation
    PopSize = PopSize1 + PopSize2;     % Total population size

    % Initialize population
    x = repmat(lb, PopSize, 1) + repmat((ub - lb), PopSize, 1) .* rand(PopSize, n);
    xold = x;

    % Evaluate initial population
    fitx = zeros(PopSize, 1);
    for i = 1:PopSize
        fitx(i) = fobj(x(i, :));
    end
    current_eval = current_eval + PopSize;

    % Initialize convergence curve
    Cg_curve = zeros(1, T);

    % Initialize best solution
    [bestfit, bes_l] = min(fitx);
    bestpos = x(bes_l, :);

    % Record initial best fitness
    Cg_curve(1) = bestfit;

    %% Initialize subpopulations and parameters
    EA_1 = x(1:PopSize1, :);
    EA_obj1 = fitx(1:PopSize1);
    EA_1old = xold(1:PopSize1, :);

    EA_2 = x(PopSize1+1:end, :);
    EA_obj2 = fitx(PopSize1+1:end);

    % Initialize CMA-ES parameters
    setting = [];
    bnd = [];
    fitness = [];
    [setting] = init_cma_par(setting, EA_2, n, PopSize2);

    % Initialize probabilities
    probDE1 = ones(1, 2) / 2;
    probSC = ones(1, 2) / 2;

    % Initialize archive
    arch_rate = 2.6;
    archive.NP = round(arch_rate * PopSize1);  % Maximum size of the archive
    archive.pop = zeros(0, n);                 % Solutions stored in the archive
    archive.funvalues = zeros(0, 1);           % Function values of archived solutions

    % Initialize parameters for adaptation
    hist_pos = 1;
    memory_size = 6;
    archive_f = ones(1, memory_size) * 0.7;
    archive_Cr = ones(1, memory_size) * 0.5;
    archive_T = ones(1, memory_size) * 0.1;
    archive_freq = ones(1, memory_size) * 0.5;

    % Initialize other parameters
    stop_con = 0;
    InitPop = PopSize1;
    thrshold = 1e-08;

    cy = 0;
    indx = 0;
    Probs = ones(1, 2);

    %% Main loop
    while iter < T && current_eval < Max_FES
        iter = iter + 1;
        cy = cy + 1;  % Control variable for CS

        % Determine the best phase
        if cy == ceil(5 + 1)  % Par.CS = 5, hardcoded for simplicity
            % Compute normalized quality and diversity
            qual = [EA_obj1(1), EA_obj2(1)];
            norm_qual = qual / sum(qual);
            norm_qual = 1 - norm_qual;  % Bigger is better

            D1 = mean(pdist2(EA_1(2:end, :), EA_1(1, :)));
            D2 = mean(pdist2(EA_2(2:end, :), EA_2(1, :)));
            norm_div = [D1, D2] / sum([D1, D2]);

            % Total improvement
            Probs = norm_qual + norm_div;
            % Update probabilities
            Probs = max(0.1, min(0.9, Probs / sum(Probs)));

            [~, indx] = max(Probs);
            if Probs(1) == Probs(2)
                indx = 0;  % No sharing of information
            end
        elseif cy == 2 * ceil(5)
            % Share information
            if indx == 1
                list_ind = randperm(PopSize1);
                list_ind = list_ind(1:min(PopSize2, PopSize1));
                EA_2(1:length(list_ind), :) = EA_1(list_ind, :);
                EA_obj2(1:length(list_ind)) = EA_obj1(list_ind);
                [setting] = init_cma_par(setting, EA_2, n, PopSize2);
                setting.sigma = setting.sigma * (1 - (current_eval / Max_FES));
            else
                if all(EA_2(1, :) >= lb) && all(EA_2(1, :) <= ub)
                    EA_1(PopSize1, :) = EA_2(1, :);
                    EA_obj1(PopSize1) = EA_obj2(1);
                    [EA_obj1, ind] = sort(EA_obj1);
                    EA_1 = EA_1(ind, :);
                end
            end
            cy = 1;
            Probs = ones(1, 2);
        end

        % EBO phase
        if current_eval < Max_FES && rand < Probs(1)
            % Population size reduction
            UpdPopSize = round((((5 - InitPop) / Max_FES) * current_eval) + InitPop);  % Par.MinPopSize = 5
            if PopSize1 > UpdPopSize
                reduction_ind_num = PopSize1 - UpdPopSize;
                if PopSize1 - reduction_ind_num < 5
                    reduction_ind_num = PopSize1 - 5;
                end
                % Remove the worst individuals
                for r = 1:reduction_ind_num
                    EA_1(end, :) = [];
                    EA_1old(end, :) = [];
                    EA_obj1(end) = [];
                    PopSize1 = PopSize1 - 1;
                end
                archive.NP = round(arch_rate * PopSize1);
                if size(archive.pop, 1) > archive.NP
                    rndpos = randperm(size(archive.pop, 1));
                    rndpos = rndpos(1:archive.NP);
                    archive.pop = archive.pop(rndpos, :);
                    archive.funvalues = archive.funvalues(rndpos);
                end
            end

            % Apply EBO
            [EA_1, EA_1old, EA_obj1, probDE1, bestfit, bestpos, archive, hist_pos, memory_size, archive_f, archive_Cr, archive_T, archive_freq, current_eval] = ...
                EBO(EA_1, EA_1old, EA_obj1, probDE1, bestfit, bestpos, archive, hist_pos, memory_size, archive_f, archive_Cr, archive_T, ...
                archive_freq, lb, ub, n, PopSize1, current_eval, fobj, Max_FES, T, iter);
        end

        % Scout phase (CMA-ES)
        if current_eval < Max_FES && rand < Probs(2)
            [EA_2, EA_obj2, setting, bestfit, bestpos, bnd, fitness, current_eval] = ...
                Scout(EA_2, EA_obj2, probSC, setting, iter, bestfit, bestpos, fitness, bnd, lb, ub, n, PopSize2, current_eval, fobj);
        end

        % Record best fitness
        Cg_curve(iter) = bestfit;

        % Check stopping condition
        if current_eval >= Max_FES
            stop_con = 1;
        end

        % Optional: Check if solution is acceptable (e.g., fitness below threshold)
        if abs(bestfit) <= thrshold
            stop_con = 1;
        end
    end
end












%% Helper Functions

function [x, xold, fitx, prob, bestfit, bestpos, archive, hist_pos, memory_size, archive_f, archive_Cr, archive_T, archive_freq, current_eval] = ...
    EBO(x, xold, fitx, prob, bestfit, bestpos, archive, hist_pos, memory_size, archive_f, archive_Cr, archive_T, ...
    archive_freq, lb, ub, n, PopSize, current_eval, fobj, Max_FES, G_Max, gg)
% EBO: Exploration Butterfly Optimization phase

vi = zeros(PopSize, n);

% Calculate CR and F
mem_rand_index = ceil(memory_size * rand(PopSize, 1));
mu_sf = archive_f(mem_rand_index);
mu_cr = archive_Cr(mem_rand_index);
mu_T = archive_T(mem_rand_index);
mu_freq = archive_freq(mem_rand_index);

% Generate CR
cr = (mu_cr + 0.1 * sqrt(pi) .* (asin(-rand(1, PopSize)) + asin(rand(1, PopSize))))';
cr(mu_cr == -1) = 0;
cr = min(cr, 1);
cr = max(cr, 0);

% Generate F
F = mu_sf + 0.1 * tan(pi * (rand(1, PopSize) - 0.5));
pos = find(F <= 0);
while ~isempty(pos)
    F(pos) = mu_sf(pos) + 0.1 * tan(pi * (rand(1, length(pos)) - 0.5));
    pos = find(F <= 0);
end
F = min(F, 1)';

% Generate T
T = mu_T + 0.05 * (sqrt(pi) .* (asin(-rand(1, PopSize)) + asin(rand(1, PopSize))));
T = max(T, 0)';
T = min(T, 0.5)';
l = floor(n * rand(1, PopSize)) + 1;

% Generate freq
freq = mu_freq + 0.1 * tan(pi * (rand(1, PopSize) - 0.5));
pos_f = find(freq <= 0);
while ~isempty(pos_f)
    freq(pos_f) = mu_freq(pos_f) + 0.1 * tan(pi * (rand(1, length(pos_f)) - 0.5));
    pos_f = find(freq <= 0);
end
freq = min(freq, 1)';

% Mutation and crossover
popAll = [x; archive.pop];  % Set archive
NP2 = size(popAll, 1);
r0 = (1:PopSize)';

% Generate random integer numbers
[r1, r2, r3] = gnR1R2_current(PopSize, PopSize, NP2, r0);

% Mutation strategies
bb = rand(PopSize, 1);
probiter = prob(1, :);
l2 = sum(prob(1:2));
op_1 = bb <= probiter(1);
op_2 = bb > probiter(1) & bb <= l2;

pNP = max(round(0.1 * PopSize), 2);  % Choose at least two best solutions
randindex = ceil(rand(1, PopSize) .* pNP);
randindex = max(1, randindex);
phix = x(randindex, :);

% Ensure indices are within bounds
idx_op1 = find(op_1 == 1);
idx_op2 = find(op_2 == 1);

vi(idx_op1, :) = x(idx_op1, :) + F(idx_op1, ones(1, n)) .* ...
    (x(r1(idx_op1), :) - x(idx_op1, :) + x(r3(idx_op1), :) - popAll(r2(idx_op1), :));

vi(idx_op2, :) = x(idx_op2, :) + F(idx_op2, ones(1, n)) .* ...
    (phix(idx_op2, :) - x(idx_op2, :) + x(r1(idx_op2), :) - x(r3(idx_op2), :));

% Handle boundaries
vi = han_boun(vi, ub, lb, x, PopSize, 1);

% Crossover
cr_matrix = cr(:, ones(1, n));
mask = rand(PopSize, n) > cr_matrix;
rows = (1:PopSize)';
cols = floor(rand(PopSize, 1) * n) + 1;
jrand = sub2ind([PopSize n], rows, cols);
mask(jrand) = false;
ui = vi;
ui(mask) = x(mask);

% Evaluate
fitx_new = zeros(PopSize, 1);
for i = 1:PopSize
    fitx_new(i) = fobj(ui(i, :));
end
current_eval = current_eval + PopSize;

% Selection
diff = abs(fitx - fitx_new);
I = (fitx_new < fitx);
goodCR = cr(I == 1);
goodF = F(I == 1);
goodT = T(I == 1)';
goodFreq = freq(I == 1);

% Update archive
archive = updateArchive(archive, x(I == 1, :), fitx(I == 1));

% Update probabilities
diff2 = max(0, (fitx - fitx_new)) ./ abs(fitx);
count_S(1) = max(0, mean(diff2(op_1 == 1)));
count_S(2) = max(0, mean(diff2(op_2 == 1)));
if sum(count_S) ~= 0
    prob = max(0.1, min(0.9, count_S ./ sum(count_S)));
else
    prob = ones(1, 2) / 2;
end

% Update population
fitx(I == 1) = fitx_new(I == 1);
xold(I == 1, :) = x(I == 1, :);
x(I == 1, :) = ui(I == 1, :);

% Update memory
num_success_params = numel(goodCR);
if num_success_params > 0
    weightsDE = diff(I == 1) ./ sum(diff(I == 1));

    % Ensure weightsDE is a column vector
    weightsDE = weightsDE(:);

    % Update memory of scaling factor
    archive_f(hist_pos) = sum(weightsDE .* (goodF .^ 2)) / sum(weightsDE .* goodF);

    % Update memory of crossover rate
    if max(goodCR) == 0 || archive_Cr(hist_pos) == -1
        archive_Cr(hist_pos) = -1;
    else
        archive_Cr(hist_pos) = sum(weightsDE .* (goodCR .^ 2)) / sum(weightsDE .* goodCR);
    end

    % Update memory of T
    archive_T(hist_pos) = sum(weightsDE .* (goodT .^ 2)) / sum(weightsDE .* goodT);

    % Update memory of freq
    if max(goodFreq) == 0 || archive_freq(hist_pos) == -1
        archive_freq(hist_pos) = -1;
    else
        archive_freq(hist_pos) = sum(weightsDE .* (goodFreq .^ 2)) / sum(weightsDE .* goodFreq);
    end

    % Update hist_pos
    hist_pos = hist_pos + 1;
    if hist_pos > memory_size
        hist_pos = 1;
    end
end

% Sort population
[fitx, ind] = sort(fitx);
x = x(ind, :);
xold = xold(ind, :);

% Update best solution
if fitx(1) < bestfit && all(x(1, :) >= lb) && all(x(1, :) <= ub)
    bestfit = fitx(1);
    bestpos = x(1, :);
end
end

function [r1, r2, r3] = gnR1R2_current(NP1, NP_x, NP_popAll, r0)
% Generate random indices for mutation operators
r1 = zeros(NP1, 1);
r2 = zeros(NP1, 1);
r3 = zeros(NP1, 1);

for i = 1:NP1
    % For r1 and r3 (indices into x)
    candidates_x = 1:NP_x;
    candidates_x(r0(i)) = [];  % Exclude current index
    perm_x = randperm(length(candidates_x));
    r1(i) = candidates_x(perm_x(1));
    r3(i) = candidates_x(perm_x(2));

    % For r2 (index into popAll)
    candidates_popAll = 1:NP_popAll;
    perm_popAll = randperm(length(candidates_popAll));
    r2(i) = candidates_popAll(perm_popAll(1));
end
end

function vi = han_boun(vi, ub, lb, x, PopSize, method)
% Handle boundaries
for i = 1:PopSize
    for j = 1:length(lb)
        if vi(i, j) < lb(j)
            vi(i, j) = lb(j) + rand * (x(i, j) - lb(j));
        elseif vi(i, j) > ub(j)
            vi(i, j) = ub(j) - rand * (ub(j) - x(i, j));
        end
    end
end
end

function archive = updateArchive(archive, pop, funvalues)
% Update archive with new solutions

% Ensure that funvalues is a column vector
if isrow(funvalues)
    funvalues = funvalues';
end

% Concatenate the current archive with the new population
combined_pop = [archive.pop; pop];
combined_funvalues = [archive.funvalues; funvalues];

% Remove any NaN or Inf values
valid_indices = all(isfinite(combined_pop), 2) & isfinite(combined_funvalues);
combined_pop = combined_pop(valid_indices, :);
combined_funvalues = combined_funvalues(valid_indices);

% Remove duplicate solutions
[unique_pop, ia, ~] = unique(combined_pop, 'rows', 'stable');
unique_funvalues = combined_funvalues(ia);

% If the archive exceeds the maximum size, truncate it
if size(unique_pop, 1) > archive.NP
    indices = randperm(size(unique_pop, 1));
    indices = indices(1:archive.NP);
    archive.pop = unique_pop(indices, :);
    archive.funvalues = unique_funvalues(indices);
else
    archive.pop = unique_pop;
    archive.funvalues = unique_funvalues;
end
end

function [setting] = init_cma_par(setting, EA_2, n, PopSize2)
% Initialize CMA-ES parameters
setting.sigma = 0.3;
setting.mu = floor(PopSize2 / 2);
setting.weights = log(setting.mu + 0.5) - log(1:setting.mu)';
setting.weights = setting.weights / sum(setting.weights);
setting.mueff = 1 / sum(setting.weights .^ 2);
setting.cc = (4 + setting.mueff / n) / (n + 4 + 2 * setting.mueff / n);
setting.cs = (setting.mueff + 2) / (n + setting.mueff + 5);
setting.c1 = 2 / ((n + 1.3)^2 + setting.mueff);
setting.cmu = min(1 - setting.c1, 2 * (setting.mueff - 2 + 1 / setting.mueff) / ((n + 2)^2 + setting.mueff));
setting.damps = 1 + 2 * max(0, sqrt((setting.mueff - 1) / (n + 1)) - 1) + setting.cs;
setting.pc = zeros(n, 1);
setting.ps = zeros(n, 1);
setting.B = eye(n);
setting.D = ones(n, 1);
setting.C = setting.B * diag(setting.D.^2) * setting.B';
setting.invsqrtC = setting.B * diag(setting.D .^ -1) * setting.B';
setting.eigeneval = 0;
setting.chiN = n^0.5 * (1 - 1 / (4 * n) + 1 / (21 * n^2));
end

function [EA_2, EA_obj2, setting, bestfit, bestpos, bnd, fitness, current_eval] = ...
    Scout(EA_2, EA_obj2, probSC, setting, iter, bestfit, bestpos, fitness, bnd, lb, ub, n, PopSize2, current_eval, fobj)
% Scout: CMA-ES phase

% CMA-ES parameters
sigma = setting.sigma;
mu = setting.mu;
weights = setting.weights;
mueff = setting.mueff;
cc = setting.cc;
cs = setting.cs;
c1 = setting.c1;
cmu = setting.cmu;
damps = setting.damps;
pc = setting.pc;
ps = setting.ps;
B = setting.B;
D = setting.D;
C = setting.C;
invsqrtC = setting.invsqrtC;
eigeneval = setting.eigeneval;
chiN = setting.chiN;

% Generate new population
arz = randn(n, PopSize2);
arx = repmat(EA_2(1, :)', 1, PopSize2) + sigma * (B * diag(D) * arz);
arx = arx';

% Handle boundaries
arx = min(ub, max(lb, arx));

% Evaluate
arfitness = zeros(PopSize2, 1);
for i = 1:PopSize2
    arfitness(i) = fobj(arx(i, :));
end
current_eval = current_eval + PopSize2;

% Selection and recombination
[arfitness, arindex] = sort(arfitness);
xold = EA_2(1, :);
EA_2 = arx(arindex(1:PopSize2), :);
EA_obj2 = arfitness(1:PopSize2);

% Update best solution
if EA_obj2(1) < bestfit && all(EA_2(1, :) >= lb) && all(EA_2(1, :) <= ub)
    bestfit = EA_obj2(1);
    bestpos = EA_2(1, :);
end

% Update evolution paths
y = (EA_2(1:mu, :) - repmat(xold, mu, 1))' / sigma;
z = B * inv(diag(D)) * y;
ps = (1 - cs) * ps + sqrt(cs * (2 - cs) * mueff) * mean(z, 2);
hsig = norm(ps) / sqrt(1 - (1 - cs)^(2 * iter / PopSize2)) / chiN < 1.4 + 2 / (n + 1);
pc = (1 - cc) * pc + hsig * sqrt(cc * (2 - cc) * mueff) * mean(y, 2);

% Adapt covariance matrix C
C = (1 - c1 - cmu) * C + c1 * (pc * pc' + (1 - hsig) * cc * (2 - cc) * C) + ...
    cmu * y * diag(weights) * y';

% Adapt step size sigma
sigma = sigma * exp((cs / damps) * (norm(ps) / chiN - 1));

% Decompose C
if iter - eigeneval > PopSize2 / (c1 + cmu) / n / 10
    eigeneval = iter;
    C = triu(C) + triu(C,1)'; % Enforce symmetry
    [B, D] = eig(C);
    D = sqrt(diag(D));
    invsqrtC = B * diag(D .^ -1) * B';
end

% Update setting
setting.sigma = sigma;
setting.pc = pc;
setting.ps = ps;
setting.B = B;
setting.D = D;
setting.C = C;
setting.invsqrtC = invsqrtC;
setting.eigeneval = eigeneval;
end
