function Positions = initialization(SearchAgents_no, dim, ub, lb)

    Boundary_no = size(ub, 2); % number of boundaries

    % Generate Hammersley sequence for initialization
    Hammersley_seq = hammersleySequence(SearchAgents_no, dim);

    Positions = zeros(SearchAgents_no, dim);

    % If the boundaries of all variables are equal and user enters a single
    % number for both ub and lb
    if Boundary_no == 1
        for i = 1:dim
            Positions(:, i) = Hammersley_seq(:, i) * (ub - lb) + lb;
        end
    end

    % If each variable has a different lb and ub
    if Boundary_no > 1
        for i = 1:dim
            ub_i = ub(i);
            lb_i = lb(i);
            Positions(:, i) = Hammersley_seq(:, i) * (ub_i - lb_i) + lb_i;
        end
    end
end

function seq = hammersleySequence(NP, dim)
    seq = zeros(NP, dim);
    for i = 1:NP
        seq(i, 1) = (i - 1) / NP; % First dimension using uniform distribution
        for j = 2:dim
            seq(i, j) = radicalInverse(i - 1, j);
        end
    end
end

function val = radicalInverse(n, base)
    val = 0;
    f = 1;
    while n > 0
        f = f / base;
        val = val + f * mod(n, base);
        n = floor(n / base);
    end
end