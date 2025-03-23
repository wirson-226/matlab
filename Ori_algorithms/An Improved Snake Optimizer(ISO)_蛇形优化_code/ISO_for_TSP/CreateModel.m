function model=CreateModel()

data = csvread('att48.csv');
x = data(:, 1);
y = data(:, 2);  
    n=numel(x); 
    d=zeros(n,n);
    for i=1:n-1
        for j=i+1:n
            d(i,j)=sqrt((x(i)-x(j))^2+(y(i)-y(j))^2);
            d(j,i)=d(i,j);
        end
    end

    xmin=min(x);
    xmax=max(x);
    ymin=min(y);
    ymax=max(y);
    model.n=n;
    model.x=x;
    model.y=y;
    model.d=d;
    model.xmin=xmin;
    model.xmax=xmax;
    model.ymin=ymin;
    model.ymax=ymax;
    
end