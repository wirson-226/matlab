function [Gb_Fit, Gb_Sol, Conv_curve] = ALSHADE(N, T, lb, ub, dim, fobj)
% ALSHADE: Adaptive Local Search SHADE Algorithm
% Inputs:
%   N    - Population size
%   T    - Maximum number of iterations
%   lb   - Lower bounds (vector of length dim)
%   ub   - Upper bounds (vector of length dim)
%   dim  - Problem dimension
%   fobj - Objective function handle
% Outputs:
%   best_fit - Best fitness value found
%   best_pos - Best position found
%   cg       - Best fitness value at each iteration

% Ensure lb and ub are column vectors
lb = lb(:);
ub = ub(:);

% Check that lb and ub are of length dim
if length(lb) ~= dim || length(ub) ~= dim
    error('Length of lb and ub must be equal to dim.');
end

% Initialize parameters
F = 0.5;     % Initial scaling factor
CR = 0.5;    % Initial crossover rate
rarc = 2.6;  % External archive size factor
p = 0.11;    % p-best selection rate
H = 6;       % Size of the historical memory
NPinit = N;  % Initial population size
NPmin = 4;   % Minimum population size
FEs = 0;     % Function evaluation counter
P = 0.5;     % Probability for mutation strategy

% Initialize population
X = lb + (ub - lb) .* rand(dim, N);
fitness = zeros(1, N);
for i = 1:N
    fitness(i) = fobj(X(:, i)');
end
FEs = FEs + N;

% Sort population based on fitness
[fitness, fidx] = sort(fitness);
X = X(:, fidx);

% Initialize archive
Asize = round(rarc * N); % Archive size
A = X(:, 1);             % Initialize archive with the best individual
Afitness = fitness(1);
nA = 1;
MF = F * ones(H, 1);     % Memory of scaling factors
MCR = CR * ones(H, 1);   % Memory of crossover rates
MCR(H) = 0.9;
MF(H) = 0.9;
iM = 1;

% Initialize variables
V = X;
U = X;
S_CR = zeros(1, N); % Set of crossover rates
S_F = zeros(1, N);  % Set of scaling factors
S_df = zeros(1, N); % Set of fitness differences

% Generate random numbers from the Cauchy distribution
Chy = cauchyrnd(0, 0.1, [1, N + 200]);
iChy = 1;

% Initialize convergence curve
Conv_curve = zeros(1, T);

% Main loop
for iter = 1:T
    SEL = ceil(nA / 2);
    weights = log(SEL + 0.5) - log(1:SEL)';
    weights = weights / sum(weights);
    Xsel = A(:, 1:SEL);
    xmean = Xsel * weights;

    % p-best index
    pbest = 1 + floor(max(2, round(p * N)) * rand(1, N));

    % Memory indices
    r = floor(1 + H * rand(1, N));

    % Crossover rates
    CR = MCR(r)' + 0.1 * randn(1, N);
    CR((CR < 0) | (MCR(r)' == -1)) = 0;
    CR(CR > 1) = 1;

    % Scaling factors
    F = zeros(1, N);
    for i = 1:N
        while F(i) <= 0
            F(i) = MF(r(i)) + Chy(iChy);
            iChy = mod(iChy, numel(Chy)) + 1;
        end
    end
    F(F > 1) = 1;

    PA = [X, A];

    % Mutation and Crossover
    memory = zeros(1, N);
    for i = 1:N
        % Generate r1
        r1 = floor(1 + N * rand);
        while i == r1
            r1 = floor(1 + N * rand);
        end
        % Generate r2
        r2 = floor(1 + (N + nA) * rand);
        while r1 == r2
            r2 = floor(1 + (N + nA) * rand);
        end
        if rand < P
            V(:, i) = X(:, i) + F(i) .* (X(:, pbest(i)) - X(:, i)) + F(i) .* (X(:, r1) - PA(:, r2));
            memory(i) = 1;
        else
            V(:, i) = X(:, i) + F(i) .* (xmean - X(:, i)) + F(i) .* (X(:, r1) - PA(:, r2));
            memory(i) = 3;
        end
        % Boundary correction
        V(:, i) = min(max(V(:, i), lb), ub);

        % Binomial Crossover
        jrand = floor(1 + dim * rand);
        for j = 1:dim
            if rand < CR(i) || j == jrand
                U(j, i) = V(j, i);
            else
                U(j, i) = X(j, i);
            end
        end
    end

    % Evaluation
    fu = zeros(1, N);
    for i = 1:N
        fu(i) = fobj(U(:, i)');
    end
    FEs = FEs + N;

    % Selection
    elitism = fu <= fitness;
    LL = zeros(1, N);
    LL(elitism) = 1;
    LLL = memory + LL;

    A1_ALL = sum(memory == 1);
    A1_better = sum(LLL == 2);

    A2_ALL = sum(memory == 3);
    A2_better = sum(LLL == 4);

    if A1_ALL ~= 0 && A2_ALL ~= 0
        P_A1 = A1_better / A1_ALL;
        P_A2 = A2_better / A2_ALL;
        P = P + 0.05 * (1 - P) * (P_A1 - P_A2) * iter / T;
        P = min(0.9, P);
        P = max(0.1, P);
    end

    nS = 0;
    for i = 1:N
        if fu(i) < fitness(i)
            nS = nS + 1;
            S_CR(nS) = CR(i);
            S_F(nS) = F(i);
            S_df(nS) = abs(fu(i) - fitness(i));
            X(:, i) = U(:, i);
            fitness(i) = fu(i);
            if nA < Asize
                A(:, nA + 1) = X(:, i);
                Afitness(nA + 1) = fu(i);
                nA = nA + 1;
            else
                ri = floor(1 + Asize * rand);
                A(:, ri) = X(:, i);
                Afitness(ri) = fu(i);
            end
        elseif fu(i) == fitness(i)
            X(:, i) = U(:, i);
        end
    end

    % Update MCR and MF
    if nS > 0
        w = S_df(1:nS) ./ sum(S_df(1:nS));
        if all(S_CR(1:nS) == 0)
            MCR(iM) = -1;
        elseif MCR(iM) ~= -1
            MCR(iM) = sum(w .* S_CR(1:nS) .* S_CR(1:nS)) / sum(w .* S_CR(1:nS));
        end
        MF(iM) = sum(w .* S_F(1:nS) .* S_F(1:nS)) / sum(w .* S_F(1:nS));
        iM = mod(iM, H) + 1;
    end

    % Sort population
    [fitness, fidx] = sort(fitness);
    X = X(:, fidx);

    % Update population size
    N = round(NPinit - (NPinit - NPmin) * iter / T);
    N = max(N, NPmin);
    fitness = fitness(1:N);
    X = X(:, 1:N);
    U = U(:, 1:N);

    % Update archive
    [Afitness, Ax] = sort(Afitness);
    A = A(:, Ax);
    Asize = round(rarc * N);
    if nA > Asize
        nA = Asize;
        A = A(:, 1:Asize);
        Afitness = Afitness(1:Asize);
    end

    % Record convergence
    Conv_curve(iter) = fitness(1);
end

Gb_Fit = fitness(1);
Gb_Sol = X(:, 1)';

end

% Helper function to generate Cauchy random numbers
function r = cauchyrnd(a, b, sz)
% Generate Cauchy random numbers with location 'a' and scale 'b'
% sz is the size (scalar or vector) of the output array
if nargin < 3
    sz = [1, 1];
end
r = a + b * tan(pi * (rand(sz) - 0.5));
end
