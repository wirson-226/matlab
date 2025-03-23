function pop=GoodPointSet(pop_size,dimension,ub,lb)
    bounds1=lb.*ones(dimension,1);
    bounds2=ub.*ones(dimension,1);
    bounds=[bounds1,bounds2];
    p=zeros(pop_size,dimension);
    prime_number_min=dimension*2+3;
    while 1
        if isprime(prime_number_min)==1
            break;
        else
           prime_number_min=prime_number_min+1;
        end
    end
    for i=1:pop_size
        for j=1:dimension
            r=mod(2*cos(2*pi*j/prime_number_min)*i,1);
            p(i,j)=bounds(j,1)+r*(bounds(j,2)-bounds(j,1));
        end
    end
    pop=p;
end