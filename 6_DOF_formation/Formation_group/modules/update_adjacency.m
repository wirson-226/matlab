function A = update_adjacency(x, r_comm)
    N = size(x,1);
    A = zeros(N,N);
    for i = 1:N
        for j = i+1:N
            dist = norm(x(i,:) - x(j,:));
            if dist <= r_comm
                A(i,j) = 1;
                A(j,i) = 1;  % 对称通信
            end
        end
    end
end
